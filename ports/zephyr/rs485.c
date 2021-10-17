/*
 * @file
 * @brief RS-485 Interface
 *
 * Copyright (C) 2020 Steve Karg <skarg@users.sourceforge.net>
 *
 * @page License
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
 * IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY
 * CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,
 * TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */
#include "rs485.h"

#include <device.h>
#include <devicetree.h>
#include <drivers/gpio.h>
#include <drivers/uart.h>
#include <init.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "bacnet/basic/sys/fifo.h"
#include "bacnet/basic/sys/mstimer.h"
#include "bacnet/bits.h"
#include "bacnet/datalink/mstpdef.h"
// #include <modbus_internal.h>

#include <logging/log.h>
LOG_MODULE_REGISTER(bacnet_rs485, LOG_LEVEL_INF);

/* Using modbus context for handling rs485 communication */
static int rs485_iface;
const struct device *rs485_uart_dev;
const struct device *rs485_gpio_dev;

#ifdef CONFIG_SOC_NRF52840_QIAA
#define BACNET_GPIO "GPIO_1"
#define RS485_PIN_DE 10
#define RS485_PIN_RE 12
#else
#define BACNET_GPIO "GPIO_0"
#define RS485_PIN_DE 2
#define RS485_PIN_RE 5
#endif

// struct modbus_context *rs485_ctx;

/* buffer for storing received bytes - size must be power of two */
/* BACnet MAX_MPDU for MS/TP is 501 bytes */
static uint8_t Receive_Queue_Data[512];
static FIFO_BUFFER Receive_Queue;

/* buffer for storing bytes to transmit */
/* BACnet MAX_MPDU for MS/TP is 501 bytes */
static uint8_t Transmit_Queue_Data[512];
static FIFO_BUFFER Transmit_Queue;

/* baud rate of the UART interface */
static uint32_t Baud_Rate = 38400;
/* flag to track RTS status */
static volatile bool Transmitting;

/* statistics */
static volatile uint32_t RS485_Transmit_Bytes;
static volatile uint32_t RS485_Receive_Bytes;

/* amount of silence on the wire */
static struct mstimer Silence_Timer;
static bool Silence_Timer_Reported = false;
static bool Turnaround_Timer_Reported = false;

/**
 * @brief Reset the silence on the wire timer.
 */
void rs485_silence_reset(void) {
  mstimer_set(&Silence_Timer, 0);
  Silence_Timer_Reported = false;
  Turnaround_Timer_Reported = false;
}

/**
 * @brief Determine the amount of silence on the wire from the timer.
 * @param amount of time that might have elapsed
 * @return true if the amount of time has elapsed
 */
bool rs485_silence_elapsed(uint32_t interval) {
  bool result = mstimer_elapsed(&Silence_Timer) > interval;
  if (result && !Silence_Timer_Reported) {
    Silence_Timer_Reported = true;
    LOG_DBG("rs485_silence_elapsed %d", Transmitting);
  }
  return result;
}

/**
 * @brief Determine the turnaround time
 * @return amount of milliseconds
 */
static uint16_t rs485_turnaround_time(void) {
  /* delay after reception before transmitting - per MS/TP spec */
  /* wait a minimum  40 bit times since reception */
  /* at least 2 ms for errors: rounding, clock tick */
  if (Baud_Rate) {
    return (2 + ((Tturnaround * 1000UL) / Baud_Rate));
  } else {
    return 2;
  }
}

/**
 * @brief Use the silence timer to determine turnaround time
 * @return true if turnaround time has expired
 */
bool rs485_turnaround_elapsed(void) {
  bool result = (mstimer_elapsed(&Silence_Timer) > rs485_turnaround_time());
  if (result && !Turnaround_Timer_Reported) {
    Turnaround_Timer_Reported = true;
    LOG_DBG("rs485_turnaround_elapsed %d", Transmitting);
  }
  return result;
}

/**
 * @brief Determines if an error occured while receiving
 * @return true an error occurred
 */
bool rs485_receive_error(void) {
  return false;
}

static void tx_isr(void) {
  static struct net_buf *buf;

  int len;
  uint8_t c;
  if (FIFO_Count(&Transmit_Queue)) {
    // copy byte-by-byte to uart
    while (FIFO_Count(&Transmit_Queue)) {
      c = FIFO_Peek(&Transmit_Queue);
      len = uart_fifo_fill(rs485_uart_dev, &c, 1);
      if (len) {
        // remove the byte from buffer
        FIFO_Get(&Transmit_Queue);
        RS485_Transmit_Bytes += 1;
        rs485_silence_reset();
      } else {
        // couldn't copy more from FIFO to uart
        break;
      }
    }
  } else {
    rs485_rts_enable(false);
    /* disable the USART to generate interrupts on TX complete */
    uart_irq_tx_disable(rs485_uart_dev);
    /* enable the USART to generate interrupts on RX not empty */
    uart_irq_rx_enable(rs485_uart_dev);
  }


  /*
	if (!buf) {
		buf = net_buf_get(&uart_tx_queue, K_NO_WAIT);
		if (!buf) {
			uart_irq_tx_disable(rs485_uart_dev);
			return;
		}
	}

	len = uart_fifo_fill(rs485_uart_dev, buf->data, buf->len);
	net_buf_pull(buf, len);
	if (!buf->len) {
		net_buf_unref(buf);
		buf = NULL;
	}
*/
}

/**
 * @brief USARTx interrupt handler sub-routine
 */
static void rs485_uart_isr(const struct device *unused, void *user_data) {
  ARG_UNUSED(unused);
  ARG_UNUSED(user_data);

  if (!(uart_irq_rx_ready(rs485_uart_dev) ||
        uart_irq_tx_complete(rs485_uart_dev))) {
    LOG_DBG("spurious interrupt");
  }

  if (uart_irq_tx_complete(rs485_uart_dev)) {
    LOG_DBG("rs485_uart_isr tx complete interrupt");
    tx_isr();
  }

  if (uart_irq_rx_ready(rs485_uart_dev)) {
		uint8_t c;
		if (uart_fifo_read(rs485_uart_dev, &c, 1) != 1) {
			LOG_ERR("Failed to read UART");
			return;
		}
    if (!Transmitting) {
      FIFO_Put(&Receive_Queue, c);
      RS485_Receive_Bytes++;
    }
  }
}
#if 0
void USART6_IRQHandler(void)
{
    uint8_t data_byte;

    if (USART_GetITStatus(USART6, USART_IT_RXNE) != RESET) {
        /* Read one byte from the receive data register */
        data_byte = USART_ReceiveData(USART6);
        if (!Transmitting) {
            FIFO_Put(&Receive_Queue, data_byte);
            RS485_Receive_Bytes++;
        }
        USART_ClearITPendingBit(USART6, USART_IT_RXNE);
    }
    if (USART_GetITStatus(USART6, USART_IT_TXE) != RESET) {
        if (FIFO_Count(&Transmit_Queue)) {
            USART_SendData(USART6, FIFO_Get(&Transmit_Queue));
            RS485_Transmit_Bytes += 1;
            rs485_silence_reset();
        } else {
            /* disable the USART to generate interrupts on TX empty */
            USART_ITConfig(USART6, USART_IT_TXE, DISABLE);
            /* enable the USART to generate interrupts on TX complete */
            USART_ITConfig(USART6, USART_IT_TC, ENABLE);
        }
        USART_ClearITPendingBit(USART6, USART_IT_TXE);
    }
    if (USART_GetITStatus(USART6, USART_IT_TC) != RESET) {
        rs485_rts_enable(false);
        /* disable the USART to generate interrupts on TX complete */
        USART_ITConfig(USART6, USART_IT_TC, DISABLE);
        /* enable the USART to generate interrupts on RX not empty */
        USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);
        USART_ClearITPendingBit(USART6, USART_IT_TC);
    }
    /* check for errors and clear them */
    if (USART_GetFlagStatus(USART6, USART_FLAG_ORE) == SET) {
        USART_ClearFlag(USART6, USART_FLAG_ORE);
    }
    if (USART_GetFlagStatus(USART6, USART_FLAG_NE) == SET) {
        USART_ClearFlag(USART6, USART_FLAG_NE);
    }
    if (USART_GetFlagStatus(USART6, USART_FLAG_FE) == SET) {
        USART_ClearFlag(USART6, USART_FLAG_FE);
    }
    if (USART_GetFlagStatus(USART6, USART_FLAG_PE) == SET) {
        USART_ClearFlag(USART6, USART_FLAG_PE);
    }
}
#endif

/*
static void rs485_serial_tx_on(struct modbus_context *ctx)
{
	struct modbus_serial_config *cfg = ctx->cfg;

	if (cfg->de != NULL) {
		gpio_pin_set(cfg->de->port, cfg->de->pin, 1);
	}

	uart_irq_tx_enable(cfg->dev);
}

static void rs485_serial_tx_on(struct modbus_context *ctx)
{
	struct modbus_serial_config *cfg = ctx->cfg;

	uart_irq_tx_disable(cfg->dev);
	if (cfg->de != NULL) {
		gpio_pin_set(cfg->de->port, cfg->de->pin, 0);
	}
}
*/

/**
 * @brief Control the DE and /RE pins on the RS-485 transceiver
 * @param enable - true to set DE and /RE high, false to set /DE and RE low
 */
void rs485_rts_enable(bool enable) {
  if (Transmitting != enable) {
    LOG_DBG("rs485_rts_enable changed %d", enable);
  }
  Transmitting = enable;
  if (Transmitting) {
    gpio_pin_set(rs485_gpio_dev, RS485_PIN_DE, 1);
    gpio_pin_set(rs485_gpio_dev, RS485_PIN_RE, 1);
  } else {
    gpio_pin_set(rs485_gpio_dev, RS485_PIN_DE, 0);
    gpio_pin_set(rs485_gpio_dev, RS485_PIN_RE, 0);
  }
}

/**
 * @brief Determine the status of the transmit-enable line on the RS-485
 *  transceiver
 * @return true if RTS is enabled, false if RTS is disabled
 */
bool rs485_rts_enabled(void) {
  return Transmitting;
}

/**
 * @brief Return true if a byte is available
 * @param data_register - byte in this parameter if there is one available
 * @return true if a byte is available, with the byte in the parameter
 */
bool rs485_byte_available(uint8_t *data_register) {
#if 0
  if (!rs485_uart_dev) {
    LOG_ERR("BACNET read UART device not initialized!");
    return false;
  }
  int rx = uart_fifo_read(rs485_uart_dev, data_register, 1);

  LOG_DBG("read %d req %d", rx, 1);

  return rx == 1 ? true : false;
#endif

  bool data_available = false; /* return value */
  if (!FIFO_Empty(&Receive_Queue)) {
    if (data_register) {
      *data_register = FIFO_Get(&Receive_Queue);
    }
    rs485_silence_reset();
    data_available = true;
  }
  return data_available;
}

/**
 * @brief Transmit one or more bytes on RS-485.
 * @param buffer - array of one or more bytes to transmit
 * @param nbytes - number of bytes to transmit
 * @return true if added to queue
 */
bool rs485_bytes_send(uint8_t *buffer, uint16_t nbytes) {
#if 0
  if (!rs485_uart_dev) {
    LOG_ERR("BACNET send UART device not initialized!");
    return false;
  }
  rs485_silence_reset();
  rs485_rts_enable(true);
  int len = uart_fifo_fill(rs485_uart_dev, buffer, nbytes);
  return (len > 0);
#endif
  bool status = false;
  if (buffer && (nbytes > 0)) {
    // LOG_DBG("rs485_bytes_send %d", nbytes);
    rs485_silence_reset();
    rs485_rts_enable(true);
    if (FIFO_Count(&Transmit_Queue) == 0) {
      int len = uart_fifo_fill(rs485_uart_dev, buffer, nbytes);
      /* enable the USART to generate interrupts on TX complete */
      uart_irq_tx_enable(rs485_uart_dev);
      /* disable the USART to generate interrupts on RX not empty */
      uart_irq_rx_disable(rs485_uart_dev);

      RS485_Transmit_Bytes += len;
      status = true;
      nbytes -= len;
      buffer += len;
    }
    if (nbytes > 0) {
      LOG_INF("BACNET UART FIFO full %d", nbytes);
      // add remaining bytes to FIFO
      if (FIFO_Add(&Transmit_Queue, buffer, nbytes)) {
        status = true;
        rs485_silence_reset();
      }
    }
  }
  return status;
}

/**
 * @brief Configures the baud rate of the USART
 */
static void rs485_baud_rate_configure(void) {
  struct uart_config uart_cfg;

  if (rs485_uart_dev == NULL) {
    LOG_ERR("Failed to get UART device BACNET");
    /// log_strdup(cfg->dev_name));
    return;
  }

  uart_cfg.baudrate = Baud_Rate;
  uart_cfg.flow_ctrl = UART_CFG_FLOW_CTRL_NONE;
  uart_cfg.data_bits = UART_CFG_DATA_BITS_8;
  uart_cfg.parity = UART_CFG_PARITY_NONE;
  uart_cfg.stop_bits = UART_CFG_STOP_BITS_1;

  if (uart_configure(rs485_uart_dev, &uart_cfg) != 0) {
    LOG_ERR("Failed to configure BACNET UART");
  }

#if 0
  USART_InitTypeDef USART_InitStructure;

  USART_InitStructure.USART_BaudRate = Baud_Rate;
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;
  USART_InitStructure.USART_StopBits = USART_StopBits_1;
  USART_InitStructure.USART_Parity = USART_Parity_No;
  USART_InitStructure.USART_HardwareFlowControl =
      USART_HardwareFlowControl_None;
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
  /* Configure USARTx */
  USART_Init(USART6, &USART_InitStructure);
#endif
}

/**
 * @brief Initialize the RS-485 baud rate
 * @param baudrate - RS-485 baud rate in bits per second (bps)
 * @return true if set and valid
 */
bool rs485_baud_rate_set(uint32_t baud) {
  bool valid = true;

  switch (baud) {
    case 9600:
    case 19200:
    case 38400:
    case 57600:
    case 76800:
    case 115200:
      Baud_Rate = baud;
      rs485_baud_rate_configure();
      break;
    default:
      valid = false;
      break;
  }

  return valid;
}

/**
 * @brief Return the RS-485 baud rate
 * @return baud - RS-485 baud rate in bits per second (bps)
 */
uint32_t rs485_baud_rate(void) {
  return Baud_Rate;
}

/**
 * @brief Return the RS-485 statistics for transmit bytes
 * @return number of bytes transmitted
 */
uint32_t rs485_bytes_transmitted(void) {
  return RS485_Transmit_Bytes;
}

/**
 * @brief Return the RS-485 statistics for receive bytes
 * @return number of bytes received
 */
uint32_t rs485_bytes_received(void) {
  return RS485_Receive_Bytes;
}

/**
 * @brief Initialize the USART for RS485
 */
void rs485_init(void) {
  LOG_DBG("rs485_init");

  /* initialize the Rx and Tx byte queues */
  FIFO_Init(&Receive_Queue, &Receive_Queue_Data[0],
            (unsigned)sizeof(Receive_Queue_Data));
  FIFO_Init(&Transmit_Queue, &Transmit_Queue_Data[0],
            (unsigned)sizeof(Transmit_Queue_Data));

  rs485_gpio_dev = device_get_binding(BACNET_GPIO);
  if (rs485_gpio_dev == NULL) {
		LOG_ERR("Failed to get GPIO_1 binding");
    return;
  }
  int ret;
  ret = gpio_pin_configure(rs485_gpio_dev, RS485_PIN_DE, GPIO_OUTPUT_ACTIVE);
  if (ret < 0) {
		LOG_ERR("Failed to configure RS485_PIN_DE %d", ret);
    return;
  }
  ret = gpio_pin_configure(rs485_gpio_dev, RS485_PIN_RE, GPIO_OUTPUT_ACTIVE);
  if (ret < 0) {
		LOG_ERR("Failed to configure RS485_PIN_RE %d", ret);
    return;
  }

  LOG_DBG("Getting BACNET binding");
  rs485_uart_dev = device_get_binding("BACNET");
  uart_irq_rx_disable(rs485_uart_dev);
  uart_irq_tx_disable(rs485_uart_dev);

  uart_irq_callback_set(rs485_uart_dev, rs485_uart_isr);

  uart_irq_rx_enable(rs485_uart_dev);
  uart_irq_tx_enable(rs485_uart_dev);

  rs485_baud_rate_set(Baud_Rate);

  rs485_silence_reset();

#if 0
  GPIO_InitTypeDef GPIO_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  /* initialize the Rx and Tx byte queues */
  FIFO_Init(&Receive_Queue, &Receive_Queue_Data[0],
            (unsigned)sizeof(Receive_Queue_Data));
  FIFO_Init(&Transmit_Queue, &Transmit_Queue_Data[0],
            (unsigned)sizeof(Transmit_Queue_Data));

  /* DFR0259 RS485 Shield - TXD=PG14, RXD=PG9, USART6 */
  /* Enable GPIOx clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
  /* Enable USARTx Clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

  /* Configure USARTx Rx and Tx pins for Alternate Function (AF) */
  /* DFR0259 RS485 Shield - TXD=PG14, RXD=PG9 */
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_14;
  /* GPIO_Speed_2MHz/GPIO_Speed_25MHz/GPIO_Speed_50MHz/GPIO_Speed_100MHz */
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  /* GPIO_Mode_IN/GPIO_Mode_OUT/GPIO_Mode_AF/GPIO_Mode_AN */
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
  /* GPIO_OType_PP/GPIO_OType_OD */
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  /* GPIO_PuPd_NOPULL, GPIO_PuPd_UP, GPIO_PuPd_DOWN */
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
  GPIO_Init(GPIOG, &GPIO_InitStructure);
  /* alternate function (AF) */
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6);
  GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6);

  /* DFR0259 RS485 Shield - CE=PF15 */
  /* Enable GPIOx clock */
  RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOF, ENABLE);
  /* Configure the Request To Send (RTS) aka Transmit Enable pin */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
  GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
  GPIO_Init(GPIOF, &GPIO_InitStructure);

  /* Configure the NVIC Preemption Priority Bits */
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_0);
  /* Enable the USARTx Interrupt */
  NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);
  /* enable the USART to generate interrupts on RX */
  USART_ITConfig(USART6, USART_IT_RXNE, ENABLE);

  rs485_baud_rate_set(Baud_Rate);

  USART_Cmd(USART6, ENABLE);

  rs485_silence_reset();
#endif
}
