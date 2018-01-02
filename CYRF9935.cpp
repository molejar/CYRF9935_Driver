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
 */

#include "CYRF9935.h"

#if(CYRF9935_RX_FIFO_DISIRQ)
#define fifo_enable_irq()  __enable_irq()
#define fifo_disable_irq() __disable_irq()
#else
#define fifo_enable_irq()
#define fifo_disable_irq()
#endif

/* The CYRF9935 SPI Commands */
#define CYRF9935_SPI_CMD_RD_REG                 0x00
#define CYRF9935_SPI_CMD_WR_REG                 0x40
#define CYRF9935_SPI_CMD_RD_RX_PAY              0x80
#define CYRF9935_SPI_CMD_WR_TX_PAY(pwr, pipe)  (0xA0|((pwr) << 4)|(pipe))
#define CYRF9935_SPI_CMD_FLUSH_RX               0xC4
#define CYRF9935_SPI_CMD_FLUSH_TX               0xC5
#define CYRF9935_SPI_CMD_REUSE_TX_PL            0xD0
#define CYRF9935_SPI_CMD_WR_ACK_PAY(pwr, pipe) (0xE0|((pwr) << 3)|(pipe))
#define CYRF9935_SPI_CMD_NOP                    0xFF

/* The CYRF9935 Registers collection */
#define CYRF9935_REG_CHANNEL                    0x00
#define CYRF9935_REG_STATUS                     0x01
#define CYRF9935_REG_CONFIG1                    0x02
#define CYRF9935_REG_CONFIG2                    0x03
#define CYRF9935_REG_EN_PIPE                    0x04
#define CYRF9935_REG_EN_AA                      0x05
#define CYRF9935_REG_EN_DP                      0x06
#define CYRF9935_REG_AAWD                       0x07
#define CYRF9935_REG_ARSC                       0x08
#define CYRF9935_REG_GP_ADDR0                   0x09
#define CYRF9935_REG_GP_ADDR1                   0x0A
#define CYRF9935_REG_GP_ADDR2                   0x0B
#define CYRF9935_REG_GP_ADDR3                   0x0C
#define CYRF9935_REG_ADDR_DEV                   0x0D
#define CYRF9935_REG_ADDR_P1                    0x0E
#define CYRF9935_REG_ADDR_P2                    0x0F
#define CYRF9935_REG_ADDR_P3                    0x10
#define CYRF9935_REG_ADDR_P4                    0x11
#define CYRF9935_REG_ADDR_P5                    0x12
#define CYRF9935_REG_ADDR_P6                    0x13
#define CYRF9935_REG_ADDR_P7                    0x14
#define CYRF9935_REG_ADDR_P8                    0x15
#define CYRF9935_REG_PKGLEN_P1                  0x16
#define CYRF9935_REG_PKGLEN_P2                  0x17
#define CYRF9935_REG_PKGLEN_P3                  0x18
#define CYRF9935_REG_PKGLEN_P4                  0x19
#define CYRF9935_REG_PKGLEN_P5                  0x1A
#define CYRF9935_REG_PKGLEN_P6                  0x1B
#define CYRF9935_REG_PKGLEN_P7                  0x1C
#define CYRF9935_REG_PKGLEN_P8                  0x1D
#define CYRF9935_REG_TX_FIFO                    0x1E
#define CYRF9935_REG_RX_FIFO                    0x1F
#define CYRF9935_REG_RSSI                       0x20
#define CYRF9935_REG_STATE                      0x22
#define CYRF9935_REG_CONFIG3                    0x23
#define CYRF9935_REG_TX_FIFO_STA_SEL            0x28
#define CYRF9935_REG_DRAFT                      0x3C
#define CYRF9935_REG_CHIP_ID                    0x3D
#define CYRF9935_REG_INDIR_ADDR                 0x3E
#define CYRF9935_REG_INDIR_DATA                 0x3F

/* STATUS */
#define CYRF9935_STATUS_RX_DR                   (1 << 5)
#define CYRF9935_STATUS_TX_DS                   (1 << 4)
#define CYRF9935_STATUS_MAX_RT                  (1 << 3)
#define CYRF9935_STATUS_RSSI_REF_DONE           (1 << 2)
#define CYRF9935_STATUS_TX_FIFO_STATE           (1 << 1)
#define CYRF9935_STATUS_RX_FIFO_NOT_EMPTY       (1 << 0)
/* CONFIG1 */
#define CYRF9935_CONFIG1_IRQ_LVL_MASK           (1 << 7)
#define CYRF9935_CONFIG1_RX_DR_IRQEN            (1 << 5)
#define CYRF9935_CONFIG1_TX_DS_IRQEN            (1 << 4)
#define CYRF9935_CONFIG1_MAX_RT_IRQEN           (1 << 3)
#define CYRF9935_CONFIG1_TX_FIFO_IRQEN          (1 << 2)
#define CYRF9935_CONFIG1_RX_FIFO_IRQEN          (1 << 1)
#define CYRF9935_CONFIG1_RSSI_IRQEN             (1 << 0)
/* CONFIG2 */
#define CYRF9935_CONFIG2_ADDRLEN_MASK           (0x03 << 5)
#define CYRF9935_CONFIG2_DR_MASK                (0x01 << 4)
#define CYRF9935_CONFIG2_TX_PWR_MASK            (0x03 << 2)
#define CYRF9935_CONFIG2_CRC_EN                 (0x01 << 1)
#define CYRF9935_CONFIG2_CRC_LNG_MASK           (0x01)
/* CONFIG3 */
#define CYRF9935_CONFIG3_PCEN                   (1 << 1)
#define CYRF9935_CONFIG3_BCEN                   0x01
/* RSSI */
#define CYRF9935_RSSI_AUTO_EN                   (1 << 5)
#define CYRF9935_RSSI_REFRESH                   (1 << 4)
/* FIFO STATE */
#define CYRF9935_FIFO_EMPTY                     0x00
#define CYRF9935_FIFO_FULL                      0x01
#define CYRF9935_FIFO_NOT_EMPTY                 0x02
#define CYRF9935_FIFO_NOT_FULL                  0x03
/* INDIRECT REG ADDR */
#define CYRF9935_INDIREC_PAL                    0x01
#define CYRF9935_INDIREC_PA4DBM                 0x04

/* RX FIFO Buffer Declaration */
static uint8_t rxBuffer[CYRF9935_RX_FIFO_SIZE];

/* Default configuration */
static const CYRF9935::conf_t defConf = {
  /* .crc        = */ CYRF9935::CRC_16bit,
  /* .preamble   = */ CYRF9935::PRLEN_16bit,

  /* .txPwr      = */ CYRF9935::TXPWR_Plus_0dBm,
  /* .dataRate   = */ CYRF9935::DRATE_2MBps,
  /* .channel    = */ 1,    /* Max: 127 */

  /* .fifoSize   = */ CYRF9935::FSIZE_32,
  /* .plSize     = */ 32,   /* Min: 1; Max: 16/32; 0 for Dynamic PL */

  /* .ackMode    = */ CYRF9935::ACK_Enabled,
  /* .ackDelay   = */ 500,  /* Max: 4000 us */
  /* .ackRsDelay = */ 500,  /* Max: 4000 us */
  /* .ackCount   = */ 3,    /* Max: 16 */

  /* .gAddrWidth = */ 4,    /* Min: 2; Max: 4 */
  /* .grpAddr    = */ 0xE7E7E7E7,
  /* .devAddr    = */ 0x11,

  /* .onPipes    = */ 8,    /* Min: 1; Max: 8 */
  /* .pipeAddr   = */ {0xC1, 0xC2, 0xC3, 0xC4, 0xC5, 0xC6, 0xC7, 0xC8}
};


CYRF9935::CYRF9935(PinName mosi,
                   PinName miso,
                   PinName sck,
                   PinName csn,
                   PinName mode,
                   PinName rst,
                   PinName irq) : m_spi(mosi, miso, sck), m_cs(csn), m_mode(mode), m_rst(rst), m_irq(irq)
{
  m_conf      = NULL;
  m_cbFunc    = NULL;
  m_cbMask    = 0;
  m_rxErr     = 0;
  m_txErr     = 0;
  m_arcnt     = 0;
  m_opMode    = MODE_IDLE;
  m_useIrqPin = (irq != NC) ? true : false;
  // RX FIFO Buffer initialization
  m_pBuff     = rxBuffer;
  m_pBuffRdI  = m_pBuff;
  m_pBuffWrI  = m_pBuff;
  m_buffSize  = CYRF9935_RX_FIFO_SIZE;
  m_buffFree  = CYRF9935_RX_FIFO_SIZE;
  m_buffPlCnt = 0;
  // TX Control
  m_pTxIndex  = NULL;
  m_txPipe    = 0;
  m_txLen     = 0;
  m_txCh      = 0;
  // Configure communication bus
  m_mode = 0;               // Disable RX Mode
  m_rst = 1;                // Release RESET
  m_cs = 1;                 // Release SPI CS
  m_spi.frequency(8000000); // Set SPI speed (max. 8MHz for CYRF9935)
  m_spi.format(8,0);        // Set SPI format (8-bit, ClockPhase = 0, ClockPolarity = 0)
}


bool CYRF9935::reset(void)
{
  uint8_t regval;

  // Reset internal variables
  m_rxErr  = 0;
  m_txErr  = 0;
  m_txLen  = 0;
  m_opMode = MODE_IDLE;

  // HW Reset of CYRF9935
  m_rst = 0;
  wait_ms(5);
  m_rst = 1;
  wait_ms(20);

  // Check Chip ID
  rd_reg(CYRF9935_REG_CHIP_ID, &regval);
  if(regval != CHIP_ID) {
    return false;
  }

  // Configure IRQ
  regval = 0;
  if(m_useIrqPin) {
    regval |= (CYRF9935_CONFIG1_RX_DR_IRQEN  |
               CYRF9935_CONFIG1_TX_DS_IRQEN  |
               CYRF9935_CONFIG1_MAX_RT_IRQEN);
               //CYRF9935_CONFIG1_TX_FIFO_IRQEN);
  }
  wr_reg(CYRF9935_REG_CONFIG1, regval);

  // Set group address width, data rate and CRC
  regval = (1 << 5);
  if(m_conf->gAddrWidth > 2) { regval = (m_conf->gAddrWidth - 1) << 5; }
  regval |= m_conf->dataRate;
  regval |= m_conf->crc;
  wr_reg(CYRF9935_REG_CONFIG2, regval);

  // Set TX power
  set_tx_power(m_conf->txPwr);

  // Set Preamble
  wr_indirec(CYRF9935_INDIREC_PAL, m_conf->preamble);

  // Set automatic retransmission delay and count
  set_auto_retrans(m_conf->ackDelay, m_conf->ackRsDelay, m_conf->ackCount);

  // Set Channel
  wr_reg(CYRF9935_REG_CHANNEL, m_conf->channel & 0x7F);

  // Set Group Address
  wr_reg(CYRF9935_REG_GP_ADDR0, (uint8_t) m_conf->grpAddr);
  wr_reg(CYRF9935_REG_GP_ADDR1, (uint8_t) (m_conf->grpAddr >> 8));
  wr_reg(CYRF9935_REG_GP_ADDR2, (uint8_t) (m_conf->grpAddr >> 16));
  wr_reg(CYRF9935_REG_GP_ADDR3, (uint8_t) (m_conf->grpAddr >> 24));

  // Set Device Address
  wr_reg(CYRF9935_REG_ADDR_DEV, m_conf->devAddr);

  // Get mask from on pipes
  regval = (1 << m_conf->onPipes) - 1;

  // Enable Rx pipes
  wr_reg(CYRF9935_REG_EN_PIPE, regval);

  // Set auto ACK
  if(m_conf->ackMode  == ACK_Enabled) {
    wr_reg(CYRF9935_REG_EN_AA, regval);
  } else {
    wr_reg(CYRF9935_REG_EN_AA, 0);
  }

  // Set dynamic payload
  if((m_conf->plSize == 0) || (m_conf->ackMode == ACK_Enabled)) {
    wr_reg(CYRF9935_REG_EN_DP, regval);
  } else {
    wr_reg(CYRF9935_REG_EN_DP, 0);
  }

  // Validate plSize
  if(m_conf->plSize > m_conf->fifoSize) {
    regval = (uint8_t)m_conf->fifoSize;
  } else {
    regval = m_conf->plSize;
  }

  // Set Pipes Address and PayLoad size
  for(int i = 0; i < m_conf->onPipes; i++) {
    wr_reg(CYRF9935_REG_ADDR_P1 + i, m_conf->pipeAddr[i]);
    wr_reg(CYRF9935_REG_PKGLEN_P1 + i, regval);
  }

  // Set RX/TX FIFO size
  regval = (m_conf->fifoSize == FSIZE_16) ? (0x01 << 6) : 0x00;
  wr_reg(CYRF9935_REG_TX_FIFO, regval);
  wr_reg(CYRF9935_REG_RX_FIFO, regval);

  // Set FIFO state
  //wr_reg(CYRF9935_REG_TX_FIFO_STA_SEL, CYRF9935_FIFO_EMPTY);
  //wr_reg(CYRF9935_REG_TX_FIFO_STA_SEL, CYRF9935_FIFO_FULL);

  // Enable RSSI Auto
  wr_reg(CYRF9935_REG_RSSI, CYRF9935_RSSI_AUTO_EN);

  // Enable Broadcast RX
  wr_reg(CYRF9935_REG_CONFIG3, 0x01);

  // Clear any pending interrupts
  wr_reg(CYRF9935_REG_STATUS, CYRF9935_STATUS_RX_DR |
                              CYRF9935_STATUS_TX_DS |
                              CYRF9935_STATUS_MAX_RT);
  // Reset SW FIFO buffers
  m_buffPlCnt = 0;
  m_buffFree = m_buffSize;
  m_pBuffRdI = m_pBuffWrI = m_pBuff;

  return true;
}

bool CYRF9935::open(const conf_t *conf)
{
  m_conf = (conf == NULL) ? &defConf : conf;

  // Wait for CYRF9935 Power-on reset
  wait_ms(100);

  if(!reset()) return false;

  if(m_useIrqPin) {
    m_irq.fall(this, &CYRF9935::pin_irq);
  }

  return true;
}

void CYRF9935::close(void)
{
  set_op_mode(MODE_SLEEP);
  if(m_useIrqPin) {
    m_irq.fall(NULL);
  }
}

uint8_t CYRF9935::get_chip_id(void)
{
  uint8_t regval;
  rd_reg(CYRF9935_REG_CHIP_ID, &regval);
  return regval;
}

void CYRF9935::set_op_mode(mode_t mode)
{
  uint8_t regval;

  if(m_opMode == mode) {
    return;
  }

  rd_reg(CYRF9935_REG_CONFIG3, &regval);
  if(m_opMode == MODE_SLEEP) {
    wr_reg(CYRF9935_REG_CONFIG3, regval & ~CYRF9935_CONFIG3_PCEN);
  }

  switch(mode)
  {
    case MODE_SLEEP:
      wr_reg(CYRF9935_REG_CONFIG3, regval | CYRF9935_CONFIG3_PCEN);
      break;

    case MODE_IDLE:
      m_mode = 0;
      break;

    case MODE_RX:
      flush_fifo(FIFO_RX);
      m_mode = 1;
      wait_us(130);
      break;

    case MODE_TX:
      flush_fifo(FIFO_TX);
      m_mode = 0;
      //wait_us(130);
      break;
  }

  m_opMode = mode;
}


#if(CYRF9935_PIPE_RTCONF)
bool CYRF9935::set_rx_pipe(uint8_t pipe, bool enable, bool autoAck, uint8_t plSize)
{
  uint8_t regval, pmask = 1 << (pipe - 1);

  // Enable pipe
  rd_reg(CYRF9935_REG_EN_PIPE, &regval);
  if(enable) { regval |=  pmask; }
  else       { regval &= ~pmask; }
  wr_reg(CYRF9935_REG_EN_PIPE, regval);

  // Return if just disable
  if(!enable) return true;

  // Set Payload size
  if(plSize == 0xFF) plSize = m_conf->plSize;
  wr_reg(CYRF9935_REG_PKGLEN_P1 + pipe - 1, plSize & 0x3F);

  // Set auto ACK
  rd_reg(CYRF9935_REG_EN_AA, &regval);
  if(autoAck) { regval |=  pmask; }
  else        { regval &= ~pmask; }
  wr_reg(CYRF9935_REG_EN_AA, regval);

  // Set dynamic payload
  rd_reg(CYRF9935_REG_EN_DP, &regval);
  if(plSize == 0) { regval |=  pmask; }
  else            { regval &= ~pmask; }
  wr_reg(CYRF9935_REG_EN_DP, regval);

  return true;
}
#endif

int CYRF9935::get_enabled_pipes(void)
{
  uint8_t regval;
  rd_reg(CYRF9935_REG_EN_PIPE, &regval);
  return regval;
}

void CYRF9935::set_channel(uint8_t channel)
{
  wr_reg(CYRF9935_REG_CHANNEL, channel & 0x7F);
}

void CYRF9935::set_tx_power(txpwr_t power)
{
  uint8_t regval;

  if(power == TXPWR_Plus_4dBm) {
    wr_indirec(CYRF9935_INDIREC_PA4DBM, 0x80);
  } else {
    wr_indirec(CYRF9935_INDIREC_PA4DBM, 0x00);
    rd_reg(CYRF9935_REG_CONFIG2, &regval);
    regval &= ~CYRF9935_CONFIG2_TX_PWR_MASK;
    wr_reg(CYRF9935_REG_CONFIG2, regval | power);
  }
}

void CYRF9935::set_data_rate(drate_t rate)
{
  uint8_t regval;
  rd_reg(CYRF9935_REG_CONFIG2, &regval);
  regval &= ~CYRF9935_CONFIG2_DR_MASK;
  wr_reg(CYRF9935_REG_CONFIG2, regval | rate);
}

void CYRF9935::set_auto_retrans(uint16_t delay_us, uint16_t rsdelay_us, uint8_t count)
{
  uint8_t delay   = (delay_us / 250) & 0x0F;
  uint8_t rsdelay = (rsdelay_us / 250) & 0x0F;

  if(delay > 0) delay -= 1;
  wr_reg(CYRF9935_REG_AAWD, (delay << 4) | rsdelay);
  wr_reg(CYRF9935_REG_ARSC, count << 4);
}

void CYRF9935::enable_broadcast(bool val)
{
  uint8_t regval;
  rd_reg(CYRF9935_REG_CONFIG3, &regval);
  if(val) regval |= 0x01;
  else    regval &= ~0x01;
  wr_reg(CYRF9935_REG_CONFIG3, regval);
}

uint8_t CYRF9935::get_rssi(void)
{
  uint8_t rssi, n = 10;
  rd_reg(CYRF9935_REG_RSSI, &rssi);
  if(!(rssi & CYRF9935_RSSI_AUTO_EN)) {
    wr_reg(CYRF9935_REG_RSSI, CYRF9935_RSSI_REFRESH);
    while(n) {
      rd_reg(CYRF9935_REG_RSSI, &rssi);
      if(rssi & CYRF9935_RSSI_REFRESH) {
        wait_us(100);
        n--;
      } else {
        n = 0;
      }
    }
  }
  return ((rssi << 1) | (rssi >> 6)) & 0x1F;
}

uint8_t CYRF9935::get_state(void)
{
  uint8_t state;
  rd_reg(CYRF9935_REG_STATE, &state);
  return state;
}

void CYRF9935::flush_fifo(fifo_t val)
{
  if(val & FIFO_RX) {
    wr_reg(CYRF9935_SPI_CMD_FLUSH_RX, 0);
  }

  if(val & FIFO_TX) {
    wr_reg(CYRF9935_SPI_CMD_FLUSH_TX, 0);
  }

  if(val == FIFO_RXSW) {
    fifo_disable_irq();
    m_buffPlCnt = 0;
    m_buffFree = m_buffSize;
    m_pBuffRdI = m_pBuffWrI = m_pBuff;
    fifo_enable_irq();
  }
}

bool CYRF9935::readable(void)
{
  uint8_t status;
  bool ret;

  if(m_useIrqPin) {
    ret = (m_buffPlCnt > 0);
  } else {
    rd_reg(CYRF9935_REG_STATUS, &status);
    ret = (status & CYRF9935_STATUS_RX_DR);
  }

  return ret;
}

int CYRF9935::rx_data(uint8_t *pipe, uint8_t *rssi, uint8_t *data, uint16_t len)
{
  uint8_t frlen, plen = 0;
  uint8_t status;

  if (len == 0) return plen;

  if(m_useIrqPin) {
    if(m_buffPlCnt > 0) {
      fifo_disable_irq();
      frlen = *m_pBuffRdI;
      for(int i = 0; i < frlen; i++) {
        switch(i) {
          case 0: // FRAME Length
            break;
          case 1: // PIPE Number and RSSI Value
            *pipe =  *m_pBuffRdI & 0x0F;
            *rssi = (*m_pBuffRdI >> 4) & 0x0F;
            break;
          default: // Payload Data
            if(len > (i-2)) {
              data[i-2] = *m_pBuffRdI;
            }
            break;
        }
        m_pBuffRdI++;
        if(m_pBuffRdI >= (m_pBuff + m_buffSize)) {
          m_pBuffRdI = m_pBuff;
        }
      }
      m_buffFree += frlen;
      m_buffPlCnt--;
      fifo_enable_irq();
      plen = frlen - 2;
    }
  } else {
    rd_reg(CYRF9935_REG_STATUS, &status);
    if(status & CYRF9935_STATUS_RX_DR) {
      plen = rd_data(pipe, rssi, data, len);
    }
    wr_reg(CYRF9935_REG_STATUS,  status & 0x3C);
  }

  return plen;
}

int CYRF9935::tx_data(uint8_t pipe, const uint8_t *data, uint16_t len, bool ack, uint8_t rfch, bool pwr)
{
  int ret = 0;
  uint8_t regval, n = 200;

  // Return if not finish transmit of previous packet
  if(m_opMode == MODE_TX) {
    return -1;
  }

  // Set TX Mode
  set_op_mode(MODE_TX);

  // Use default RF channel if not set
  if(rfch == 0xFF) rfch = m_conf->channel;

  // Write payload to TX buffer
  ret = wr_data(pipe, data, len, rfch & 0x7F, pwr, ack);

  // Return, if IRQ Pin used
  if(m_useIrqPin) {
    m_txLen = len - ret;
    if(m_txLen) {
      m_txCh = rfch;
      m_txPipe = pipe;
      if(pwr) m_txPipe |= (1 << 6);
      if(ack) m_txPipe |= (1 << 7);
      m_pTxIndex = data + ret;
    }
    return 0;
  }

  // Pulling mode (if not used IRQ pin)
  while(n) {
    rd_reg(CYRF9935_REG_STATUS, &regval);

    // TX Payload NOT Transmitted
    if(regval & CYRF9935_STATUS_MAX_RT) {
      flush_fifo(FIFO_TX);
      if(m_txErr < 0xFFFFFFFF) m_txErr++;
      ret = -1;
      break;
    }

    // TX Payload Transmitted OK
    if(regval & CYRF9935_STATUS_TX_DS) {
      if(m_txErr > 0x00) m_txErr--;
      break;
    }

    wait_us(100);
    n--;
  }

  // clear any pending IRQ flags
  wr_reg(CYRF9935_REG_STATUS, 0x3C);
  m_opMode = MODE_IDLE;

  if(n == 0) {
    ret = -1;
    reset();
  }

  return ret;
}

void CYRF9935::reuse_tx_payload(uint8_t rfch, bool ack)
{
  if(!ack) rfch |= (1 << 7);
  wr_reg(CYRF9935_SPI_CMD_REUSE_TX_PL, rfch);
}

int CYRF9935::set_ack_payload(uint8_t pipe, const uint8_t *data, uint8_t len, bool pwr)
{
  uint8_t plen = (uint8_t)m_conf->fifoSize;

#if(CYRF9935_PIPE_RTCONF)
  if(pipe > 0) {
    rd_reg(CYRF9935_REG_EN_DP, &plen);
    if((plen & (1 << (pipe - 1))) == 0) {
      rd_reg(CYRF9935_REG_PKGLEN_P1 + pipe, &plen);
    }
  } else {
    if(m_conf->plSize) { plen = m_conf->plSize; }
  }
#else
  if(m_conf->plSize > 0) { plen = m_conf->plSize; }
#endif

  if(len > plen) len = plen;
  wr_reg(CYRF9935_SPI_CMD_WR_ACK_PAY(pwr ? 1 : 0, pipe), data, len);
  return len;
}

/*********************************************************************************
 * protected
 *********************************************************************************/

void CYRF9935::pin_irq(void)
{
  uint8_t status, regval;

  // Read and clear any pending IRQ flags
  rd_reg(CYRF9935_REG_STATUS, &status);
  wr_reg(CYRF9935_REG_STATUS, status & 0x3C);

  // RSSI Refresh is Done
  //if(status & CYRF9935_STATUS_RSSI_REF_DONE) {
  //}

  // TX FIFO IRQ (Empty, Full, ...)
  //if(status & CYRF9935_STATUS_TX_FIFO_STATE) {
  //}

  // TX Payload NOT Transmitted
  if(status & CYRF9935_STATUS_MAX_RT) {
    if(m_txErr < 0xFFFFFFFF) m_txErr++;
    flush_fifo(FIFO_TX);
    m_txLen  = 0;
    m_opMode = MODE_IDLE;
    status  |= EVENT_TX_DONE;
  }

  // TX Payload Transmitted OK
  if(status & CYRF9935_STATUS_TX_DS) {
    if(m_txErr > 0) m_txErr--;
    if(m_txLen) {
      // Write payload to TX buffer
      uint16_t txlen = wr_data(m_txPipe & 0x0F ,
                               m_pTxIndex,
                               m_txLen,
                               m_txCh,
                               m_txPipe & (1 << 6),
                               m_txPipe & (1 << 7));
      m_pTxIndex += txlen;
      m_txLen    -= txlen;
    } else {
      status |= EVENT_TX_DONE;
      m_opMode = MODE_IDLE;
    }
  }

  // RX Payload Received
  if(status & CYRF9935_STATUS_RX_DR) {
    uint8_t pipe, len, n = 0;
    while(n < 6) {
      rd_reg(CYRF9935_REG_RX_FIFO, &regval);
      if((regval & 0x3F) == 0) { break; }
      // Move RX data from HW FIFO to SW FIFO
      m_cs = 0;
      m_spi.write(CYRF9935_SPI_CMD_RD_RX_PAY);
      pipe = m_spi.write(CYRF9935_SPI_CMD_NOP);
      len  = m_spi.write(CYRF9935_SPI_CMD_NOP) & 0x1F; // len = 0 means 1 Byte in FIFO
      len += (1 + 2);
      if (m_buffFree >= len) {
        fifo_disable_irq();
        for(int i = 0; i < len; i++) {
          switch(i) {
            case 0: // FRAME Length
              *m_pBuffWrI = len;
              break;
            case 1: // PIPE Number and RSSI Value
              *m_pBuffWrI = pipe;
              break;
            default: // Payload Data
              *m_pBuffWrI = m_spi.write(CYRF9935_SPI_CMD_NOP);
              break;
          }
          m_pBuffWrI++;
          if(m_pBuffWrI >= (m_pBuff + m_buffSize)) {
            m_pBuffWrI = m_pBuff;
          }
        }
        m_buffFree -= len;
        m_buffPlCnt++;
        fifo_enable_irq();
      } else {
        while (len > 2) {
          m_spi.write(CYRF9935_SPI_CMD_NOP); len--;
        }
      }
      m_cs = 1;
      n++;
    }
  }

  // Call IRQ Callback
  status &= m_cbMask;
  if(m_cbFunc && status) {
    m_cbFunc(this, status);
  }
}

int CYRF9935::rd_reg(uint8_t addr, uint8_t *pVal)
{
  int status;

  m_cs = 0;
  status = m_spi.write(addr);
  *pVal = m_spi.write(CYRF9935_SPI_CMD_NOP);
  m_cs = 1;

  return status;
}

int CYRF9935::rd_reg(uint8_t addr, uint8_t *pBuff, uint8_t len)
{
  int status;

  m_cs = 0;
  status = m_spi.write(addr);
  while (len--) {
    *pBuff = m_spi.write(CYRF9935_SPI_CMD_NOP);
    pBuff++;
  }
  m_cs = 1;

  return status;
}

int CYRF9935::wr_reg(uint8_t addr, uint8_t val)
{
  int status;

  if(addr < CYRF9935_SPI_CMD_WR_REG) {
    addr |= CYRF9935_SPI_CMD_WR_REG;
  }

  m_cs = 0;
  status = m_spi.write(addr);
  m_spi.write(val);
  m_cs = 1;

  return status;
}

int CYRF9935::wr_reg(uint8_t addr, const uint8_t* pBuff, uint8_t len)
{
  int status;

  if(addr < CYRF9935_SPI_CMD_WR_REG) {
    addr |= CYRF9935_SPI_CMD_WR_REG;
  }

  m_cs = 0;
  status = m_spi.write(addr);
  while (len--) {
    m_spi.write(*pBuff);
    pBuff++;
  }
  m_cs = 1;

  return status;
}

void CYRF9935::rd_indirec(uint8_t addr, uint8_t* pVal)
{
  wr_reg(CYRF9935_REG_INDIR_ADDR, addr);
  rd_reg(CYRF9935_REG_INDIR_DATA, pVal);
}

void CYRF9935::wr_indirec(uint8_t addr, uint8_t val)
{
  wr_reg(CYRF9935_REG_INDIR_ADDR, addr);
  wr_reg(CYRF9935_REG_INDIR_DATA, val);
}

uint16_t CYRF9935::rd_data(uint8_t *pPipe, uint8_t *pRssi, uint8_t *pBuff, uint16_t len)
{
  uint8_t tmp;

  m_cs = 0;
  m_spi.write(CYRF9935_SPI_CMD_RD_RX_PAY);
  tmp = m_spi.write(CYRF9935_SPI_CMD_NOP);
  *pPipe = tmp & 0x0F;
  *pRssi = (tmp >> 4) & 0x0F;
  tmp = m_spi.write(CYRF9935_SPI_CMD_NOP) & 0x1F;
  if(len > tmp) len = tmp;
  while (len--) {
    *pBuff = m_spi.write(CYRF9935_SPI_CMD_NOP);
    pBuff++;
  }
  m_cs = 1;

  return tmp;
}

uint16_t CYRF9935::wr_data(uint8_t pipe, const uint8_t *pBuff, uint16_t len, uint8_t rfch, bool pwr, bool ack)
{
  uint16_t cnt, plen = (uint8_t) m_conf->fifoSize;
  if(ack == false) rfch |= 0x80;

#if(CYRF9935_PIPE_RTCONF)
  if(pipe > 0) {
    rd_reg(CYRF9935_REG_EN_DP, &plen);
    if((plen & (1 << (pipe - 1))) == 0) {
      rd_reg(CYRF9935_REG_PKGLEN_P1 + pipe, &plen);
    }
  } else {
    if(m_conf->plSize) { plen = m_conf->plSize; }
  }
#else
  if(m_conf->plSize) { plen = m_conf->plSize; }
#endif
  // ...
  if(len > plen) len = plen;
  cnt = len;
  // ...
  m_cs = 0;
  m_spi.write(CYRF9935_SPI_CMD_WR_TX_PAY(pwr ? 1 : 0, pipe));
  m_spi.write(rfch);
  while (cnt--) {
    m_spi.write(*pBuff);
    pBuff++;
  }
#if(1)
  if(len < plen) {
    cnt = plen - len;
    while (cnt--) { m_spi.write(0); }
  }
#endif
  m_cs = 1;

  return len;
}
