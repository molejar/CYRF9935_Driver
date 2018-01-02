/**
 * @file CYRF9935.h
 *
 * @author Martin Olejar
 *
 * @section LICENSE
 *
 * Copyright (c) 2017 Martin Olejar
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * @section DESCRIPTION
 *
 * CYRF9935 is a low power radio transceiver operating in the world wide 2.4 -
 * 2.5 GHz ISM band.
 *
 * Datasheet: http://www.cypress.com/file/139546
 */

#ifndef __CYRF9935_H__
#define __CYRF9935_H__

/**
 * Includes
 */
#include "mbed.h"

/**
 * Module configuration macros
 */

/* RX FIFO Size. Default: 204 Bytes (32+2 * 6) */
#if !defined(CYRF9935_RX_FIFO_SIZE)
#define CYRF9935_RX_FIFO_SIZE     204
#endif

/* Mask Global IRQ if read/write FIFO data
 * 0) Kept enabled
 * 1) Disable (default) */
#if !defined(CYRF9935_RX_FIFO_DISIRQ)
#define CYRF9935_RX_FIFO_DISIRQ   1
#endif

/* Runtime PIPE Configuration
 * 0) Remove
 * 1) Include */
#if !defined(CYRF9935_PIPE_RTCONF)
#define CYRF9935_PIPE_RTCONF      0
#endif

/**
 * CYRF9935 Single Chip 2.4GHz Transceiver from Cypress Semiconductor.
 */
class CYRF9935 {

public:

  enum chid_t {
    CHIP_ID           = 0xA1
  };

  enum mode_t {
    MODE_SLEEP        = 0,
    MODE_IDLE         = 1,
    MODE_RX           = 5,
    MODE_TX           = 3,
#if(0)
    /* internal modes */
    MODE_RX_READ      = 6,
    MODE_RX_ACK       = 4,
    MODE_TX_ACK       = 2
#endif
  };

  enum ack_t {
    ACK_Disabled,
    ACK_Enabled
  };

  enum crc_t {
    CRC_Disabled      = 0x00,
    CRC_8bit          = 0x02,
    CRC_16bit         = 0x03
  };

  enum txpwr_t {
    TXPWR_Plus_4dBm   = (0x01 << 7),
    TXPWR_Plus_0dBm   = (0x03 << 2),
    TXPWR_Minus_8dBm  = (0x02 << 2),
    TXPWR_Minus_14dBm = (0x01 << 2),
    TXPWR_Minus_20dBm = (0x00 << 2)
  };

  enum drate_t {
    DRATE_250kBps     = (0),
    DRATE_2MBps       = (0x01 << 4)
  };

  enum fifo_t {
    FIFO_RX           = 0x01,
    FIFO_TX           = 0x02,
    FIFO_RXTX         = 0x03,
    FIFO_RXSW         = 0x04
  };

  enum fsize_t {
    FSIZE_16          = 16,
    FSIZE_32          = 32
  };

  enum prlen_t {
    PRLEN_4bit        = 0,
    PRLEN_8bit,
    PRLEN_12bit,
    PRLEN_16bit
  };

  enum event_t {
    EVENT_MAX_RT      = (1 << 3),
    EVENT_TX_DS       = (1 << 4),
    EVENT_RX_DR       = (1 << 5),
    EVENT_TX_DONE     = (1 << 6),
    EVENT_ANY         = (3 << 3) | (1 << 6)
  };

  struct conf_t {
    crc_t    crc;
    prlen_t  preamble;

    txpwr_t  txPwr;
    drate_t  dataRate;
    uint8_t  channel;
    fsize_t  fifoSize;
    uint8_t  plSize;

    ack_t    ackMode;
    uint16_t ackDelay;
    uint16_t ackRsDelay;
    uint8_t  ackCount;

    uint8_t  gAddrWidth;
    uint32_t grpAddr;
    uint8_t  devAddr;

    uint8_t  onPipes;
    uint8_t  pipeAddr[8];
  };

  /**
   *
   * @param obj
   * @param event
   */
  typedef void (*rfcb_t)(CYRF9935 *obj, uint8_t event);

  /**
   * Constructor.
   *
   * @param mosi The MOSI pin of SPI interface.
   * @param miso The MISO pin of SPI interface.
   * @param sck  The SCK  pin of SPI interface.
   * @param csn  The chip select pin of SPI interface.
   * @param mode The control pin for Rx/Tx mode.
   * @param rst  The pin for chip reset.
   * @param irq  The pin for interrupt request from CYRF9935.
   */
  CYRF9935(PinName mosi, PinName miso, PinName sck, PinName csn, PinName mode, PinName rst, PinName irq = NC);

  /**
   * Reset CYRF9935 Device
   *
   *  @return True if successful
   */
  bool reset(void);

  /**
   * Open CYRF9935 Device
   *
   * @param conf Pointer to configuration structure
   *
   * @return True if successful
   */
  bool open(const conf_t *conf = NULL);

  /**
   * Close CYRF9935 Device
   */
  void close(void);

  /**
   * Read Chip ID (default 0xA1)
   *
   * @return value representing Chip ID
   */
  uint8_t get_chip_id(void);

  /**
   * Set the operation mode
   *
   * @param mode
   */
  void set_op_mode(mode_t mode);

  /**
   * Get info about actual operation mode
   *
   * @return The value representing actual operation mode
   */
  mode_t get_op_mode(void) { return m_opMode; };

#if(CYRF9935_PIPE_RTCONF)
  /**
   * Configure RX Pipe
   *
   * @param pipe     The pipe index (1 - 8)
   * @param enable   True or False (default: true)
   * @param autoAck  Auto acknowledge feature (default: false)
   * @param size     RX Payload size:
   *                 Use 0 for dynamic size or 1 - 32 bytes for fixed size
   *                 If use 0xFF, the value will be reused from conf_t struct. (default: 0xFF)
   */
  bool set_rx_pipe(uint8_t pipe, bool enable = true, bool autoAck = false, uint8_t plSize = 0xFF);
#endif

  /**
   * Read current configuration of pipes
   */
  int get_enabled_pipes(void);

  /**
   * Set the channel number
   *
   * @param channel
   */
  void set_channel(uint8_t channel);

  /**
   * Set the RF output power.
   *
   * @param power The RF output power in dBm (4, 0, -8, -14 or -20).
   */
  void set_tx_power(txpwr_t power);

  /**
   * Set the Air data rate.
   *
   * @param rate The air data rate (250kBps or 2MBps).
   */
  void set_data_rate(drate_t rate);

  /**
   * Set AutoRetransmit function
   *
   * @param delay The delay between restransmit's, in 250us steps  (250 - 4000)
   * @param count number of retransmits before generating an error (1..15)
   */
  void set_auto_retrans(uint16_t delay_us, uint16_t rsdelay_us, uint8_t count);

  /**
   * Enable Broadcast Message support (PIPE 0)
   *
   * @param val True = Enable, False = Disable
   */
  void enable_broadcast(bool val);

  /**
   * Get RSSI Value
   *
   * @return RSSI Value
   */
  uint8_t get_rssi(void);

  /**
   * Get CYRF9935 actual state
   *
   * @return actual state value
   */
  uint8_t get_state(void);

  /**
   * Flush RX/TX FIFO
   *
   * @param val (FIFO_RX, FIFO_TX, FIFO_RXTX or FIFO_RXSW)
   */
  void flush_fifo(fifo_t val = FIFO_RXTX);

  /**
   * Test if any RX pipe has readable data
   *
   * @return True if RX data ready
   */
  bool readable(void);

  /**
   * Read data from RX Pipe
   *
   * @param pipe the receive pipe to get data from
   * @param data pointer to an array of bytes to store the received data
   * @param len the number of bytes to read from receive buffer (1..32)
   * @return the number of bytes actually received, 0 if none are received, or -1 for an error
   */
  int rx_data(uint8_t *pipe, uint8_t *rssi, uint8_t *data, uint16_t len);

  /**
   * Transmit data
   *
   * @param data pointer to an array of bytes to write
   * @param len the number of bytes to send (1..32)
   * @param rfch
   * @return the number of bytes actually written, or -1 for an error
   */
  int tx_data(uint8_t pipe, const uint8_t *data, uint16_t len, bool ack = true, uint8_t rfch = 0xFF, bool pwr = false);

  /**
   * Register user callback function
   *
   * @param cbfunc
   * @param mask
   */
  void attach_cb(rfcb_t cbfunc, uint8_t event=EVENT_ANY) { m_cbFunc = cbfunc; m_cbMask = event; };

  /**
   * Reuse last transmitted payload.
   *
   * @param rfch
   */
  void reuse_tx_payload(uint8_t rfch, bool ack = true);

  /**
   * Set ACK payload which will transmit with every ACK message
   *
   * @param pipe Rx Pipe number (0 - 5)
   * @param data Pointer to data buffer
   * @param len Size of data buffer
   */
  int set_ack_payload(uint8_t pipe, const uint8_t *data, uint8_t len, bool pwr=true);

  /**
   *
   * @param addr
   * @param data
   * @param len
   */
  void read_regs(uint8_t addr, uint8_t *data, uint8_t len) {
    rd_reg(addr, data, len);
  };

private:
  /**
   * GPIO IRQ CallBack function
   */
  void pin_irq(void);

  /**
   * Read the content of addressable register in CYRF9935
   *
   * @param addr The address of the register
   * @param pVal The pointer to variable for read value
   * @return The content of status register
   */
  int rd_reg(uint8_t addr, uint8_t* pVal);

  /**
   * Read the content of addressable registers in CYRF9935
   *
   * @param addr The address of start register
   * @param pBuff Pointer to buffer
   * @param len Size of the buffer
   * @return The content of status register
   */
  int rd_reg(uint8_t addr, uint8_t* pBuff, uint8_t len);

  /**
   * Write the content of an addressable register in CYRF9935
   *
   * @param addr The address of the register
   * @param val  The value to be write into the register
   * @return The content of status register
   */
  int wr_reg(uint8_t addr, uint8_t val);

  /**
   * Write the content of addressable registers in CYRF9935
   *
   * @param addr The address of start register
   * @param pBuff Pointer to buffer with the content
   * @param len Size of the buffer
   * @return The content of status register
   */
  int wr_reg(uint8_t addr, const uint8_t* pBuff, uint8_t len);

  /**
   * Indirect read the content of addressable registers in CYRF9935
   *
   * @param addr The address of the register
   * @param pVal The value to be read from the register
   */
  void rd_indirec(uint8_t addr, uint8_t* pVal);

  /**
   * Indirect write the content of addressable registers in CYRF9935
   *
   * @param addr The address of the register
   * @param val  The value to be write into the register
   */
  void wr_indirec(uint8_t addr, uint8_t val);

  /**
   * Read data from the internal HW FIFO buffer
   *
   * @param pPipe
   * @param pRssi
   * @param pBuff Pointer to buffer
   * @param len
   * @return
   */
  uint16_t rd_data(uint8_t *pPipe, uint8_t *pRssi, uint8_t *pBuff, uint16_t len);

  /**
   * Write data into internal HW FIFO buffer
   *
   * @param pipe  The PIPE number (1 - 8) or 0 for broadcast message
   * @param pBuff Pointer to buffer with payload data
   * @param len   The payload size
   * @param rfch
   * @param pwr
   * @param ack   True if use ack or False if no ack
   * @return
   */
  uint16_t wr_data(uint8_t pipe, const uint8_t *pBuff, uint16_t len, uint8_t rfch, bool pwr, bool ack);


  /* SPI interface */
  SPI           m_spi;
  DigitalOut    m_cs;
  DigitalOut    m_rst;
  DigitalOut    m_mode;
  InterruptIn   m_irq;

  bool          m_useIrqPin;
  const conf_t *m_conf;

  /* RX FIFO buffer */
  uint8_t      *m_pBuff;
  uint8_t      *m_pBuffRdI;
  uint8_t      *m_pBuffWrI;
  uint16_t      m_buffPlCnt;
  uint16_t      m_buffSize;
  uint16_t      m_buffFree;

  /* TX Control */
  const uint8_t *m_pTxIndex;
  uint8_t       m_txPipe;
  uint8_t       m_txCh;
  uint16_t      m_txLen;

  /* RX CallBack function pointer */
  rfcb_t        m_cbFunc;
  uint8_t       m_cbMask;

  /* Operation Mode */
  mode_t        m_opMode;

  /* RX/TX Errors */
  uint32_t      m_rxErr;
  uint32_t      m_txErr;
  uint8_t       m_arcnt;
};

#endif /* __CYRF9935_H__ */
