#include	"rm_comm.h"


/*-- begin:	definitions --*/
#define	RM_LOG_FACTOR_MAX		32
#define	RM_SND_FRAME_BUFF_SIZE	((RM_LOG_FACTOR_MAX*4)+2)
#define	RM_SND_PAYLOAD_SIZE		(RM_SND_FRAME_BUFF_SIZE-1)

#define	RM_RCV_FRAME_BUFF_SIZE	32


#define	RM_UART_BUFF_MAX		(1 << 3)					/* buffer size  2^n (n:2-7)	*/
#define	RM_UART_BUFF_MASK		(RM_UART_BUFF_MAX - 1)




#define	RM_RCV_TIMEOUT_CNT		100
#define	RM_REQ_TIMEOUT_CNT		2000
#define	RM_SND_INTERVAL_CNT		500


/* definitions of Serial Line Internet Protocol */
#define	RM_FRAME_CHAR_END		0xC0
#define	RM_FRAME_CHAR_ESC		0xDB
#define	RM_FRAME_CHAR_ESC_END	0xDC
#define	RM_FRAME_CHAR_ESC_ESC	0xDD

/* frame */
#define	RM_INITIAL_MODE			0
#define	RM_LOG_MODE				1
#define	RM_CANDR_MODE			2


/*  */
#define	RM_FRAME_SEQCODE		0
#define	RM_FRAME_PAYLOAD		1


/* status */
#define	RM_STATUS_ERR			0
#define	RM_STATUS_SUCCESS		1

/*  */
#define	RM_SLIP_RCV_NOTHING		0
#define	RM_SLIP_RCV_INCOMING	1
#define	RM_SLIP_RCV_COMPLETE	2
#define	RM_SLIP_RCV_FAULT		3

/*  */
#define	SETLOG_BIT_MASK			0xF0
#define	SETLOG_START_BIT		0x10
#define	SETLOG_END_BIT			0x20


/*-- begin:	variables and constants --*/
#ifdef	RM_ADDRESS_4BYTE
static	uint32_t	u32_RM_DumpAddress;
static	uint8_t		u08_RM_DumpLentgh;
static	uint32_t	u32_RM_VersionAddress;
static	uint8_t		u08_RM_VersionLength;
static	uint8_t		u08_RM_SizeArray[RM_LOG_FACTOR_MAX];
static	uint32_t	u32_RM_AddressArray[RM_LOG_FACTOR_MAX];
#else
static	uint16_t	u16_RM_DumpAddress;
static	uint8_t		u08_RM_DumpLentgh;
static	uint16_t	u16_RM_VersionAddress;
static	uint8_t		u08_RM_VersionLength;
static	uint8_t		u08_RM_SizeArray[RM_LOG_FACTOR_MAX];
static	uint16_t	u16_RM_AddressArray[RM_LOG_FACTOR_MAX];
#endif

static 	uint8_t		u08_RM_CommMode;
static	uint8_t		u08_RM_ValidFlg;
static	uint8_t		u08_RM_DelayedTrg;

static	uint8_t		u08_RM_ValidSize;

static	uint8_t		u08_RM_MasCnt;
static	uint8_t		u08_RM_SlvCnt;

static	uint8_t		u08_RM_RcvBufferArray[RM_RCV_FRAME_BUFF_SIZE];
static	uint8_t		u08_RM_RcvBufferSize;

static	uint8_t		u08_RM_SndBufferArray[RM_SND_FRAME_BUFF_SIZE];
static	uint8_t		u08_RM_SndBufferSize;
static	uint8_t		u08_RM_SndBufferSentIndex;

static	uint16_t	u16_RM_RcvTimeoutCnt;
static	uint16_t	u16_RM_ReqTimeoutCnt;
static	uint16_t	u16_RM_TimingCnt;
static	uint16_t	u16_RM_WaitCnt;

static	uint8_t		u08_RM_RcvRingBuffArray[RM_UART_BUFF_MAX];
static	uint8_t		u08_RM_RcvRingBuffEndIndex;
static	uint8_t		u08_RM_RcvRingBuffBgnIndex;
static	uint8_t		u08_RM_RcvSLIPActiveFlg;

static	uint8_t		u08_RM_SndSLIPActiveFlg;


/*-- begin:	prototype of function --*/

static	uint8_t		CtrlSendTiming( uint8_t u08_mode, uint8_t u08_trg );

static	uint16_t	SetLogStart( uint8_t u08_tmp[], uint8_t u08_payloadsize );
static	uint16_t	SetLogStop( uint8_t u08_tmp[], uint8_t u08_payloadsize );
static	uint16_t	SetTiming( uint8_t u08_tmp[], uint8_t u08_payloadsize );
static	uint16_t	WriteData( uint8_t u08_tmp[], uint8_t u08_payloadsize );
static	uint16_t	SetLogData( uint8_t u08_tmp[], uint8_t u08_payloadsize );
static	uint16_t	RequestDUTCondition( uint8_t u08_tmp[], uint8_t u08_payloadsize );
static	uint16_t	SetDumpData( uint8_t u08_tmp[], uint8_t u08_payloadsize );
static	uint16_t	GetLogData( uint8_t u08_tmp[], uint8_t *u08_frame_size );
static	uint16_t	GetBlockData( uint8_t u08_tmp[], uint8_t *u08_frame_size );
static	uint8_t		GetCRC( uint8_t u08_tmp[], uint8_t u08_buff_size );
#ifdef	RM_ADDRESS_4BYTE
static	uint16_t	CheckSizeAndAddress( uint8_t u08_size, uint32_t var_address );
#else
static	uint16_t	CheckSizeAndAddress( uint8_t u08_size, uint16_t var_address );
#endif

static	uint16_t	PutSendBuff( uint8_t u08_char );
static	uint16_t	GetReceiveFrame( uint8_t u08_tmp[], uint8_t *u08_rcv_index );


/*-- begin:	functions --*/

/**
  * @brief  Main structure of RM comunication.
  * call this each timing(millie secound) which defined "RM_BASE_CYCLE"
  * @param  None
  * @retval None
  */
void	RM_Communiation( void )
{
static	uint8_t		u08_CommPayloadSize;

	uint8_t		u08_crc;

	uint8_t		u08_opcode;
	uint8_t		u08_mas_cnt;
	uint8_t		u08_payloadsize;

	uint8_t		u08_trg;
	uint8_t		u08_send_flg;

	uint16_t	u16_return_val;

	u08_trg = RM_FALSE;

	u16_return_val = GetReceiveFrame( u08_RM_RcvBufferArray, &u08_RM_RcvBufferSize );

	if( u16_return_val == RM_SLIP_RCV_FAULT )
	{
		/* Nothing to do */

	}
	else if( u16_return_val == RM_SLIP_RCV_INCOMING )
	{
		u16_RM_RcvTimeoutCnt = u16_RM_RcvTimeoutCnt + RM_BASE_CYCLE;

		if( u16_RM_RcvTimeoutCnt >= RM_RCV_TIMEOUT_CNT )
		{
			u16_RM_RcvTimeoutCnt = 0;

			u08_RM_RcvBufferSize = 0;

			u08_RM_RcvSLIPActiveFlg = RM_FALSE;

		}

	}
	else if( u16_return_val == RM_SLIP_RCV_COMPLETE )
	{
		u16_RM_RcvTimeoutCnt = 0;

		u08_opcode = u08_RM_RcvBufferArray[RM_FRAME_SEQCODE] & (uint8_t)0x0F;
		u08_mas_cnt = u08_RM_RcvBufferArray[RM_FRAME_SEQCODE] & (uint8_t)0xF0;

		/* delete opcode size */
		u08_payloadsize = (u08_RM_RcvBufferSize-1);

		u16_return_val = RM_STATUS_ERR;

		if( u08_RM_ValidFlg != RM_TRUE )
		{
			if( u08_opcode == 0x06 )
			{
				/* Condition req */
				u16_return_val = RequestDUTCondition( &u08_RM_RcvBufferArray[RM_FRAME_PAYLOAD], u08_payloadsize );

				if( u16_return_val != RM_STATUS_ERR )
				{
					u08_RM_ValidFlg = RM_TRUE;

				}

			}

		}
		else
		{
			switch( u08_opcode )
			{
			case 0x01:
				/* Start Log mode */
				u16_return_val = SetLogStart( &u08_RM_RcvBufferArray[RM_FRAME_PAYLOAD], u08_payloadsize );

				u08_RM_CommMode = RM_LOG_MODE;

				break;

			case 0x02:
				/* Stop Log mode */
				u16_return_val = SetLogStop( &u08_RM_RcvBufferArray[RM_FRAME_PAYLOAD], u08_payloadsize );

				u08_RM_CommMode = RM_CANDR_MODE;

				break;

			case 0x03:
				/* Set Timing */
				u16_return_val = SetTiming( &u08_RM_RcvBufferArray[RM_FRAME_PAYLOAD], u08_payloadsize );

				break;

			case 0x04:
				/* Write Value */
				u16_return_val = WriteData( &u08_RM_RcvBufferArray[RM_FRAME_PAYLOAD], u08_payloadsize );

				break;

			case 0x05:
				/* Set Address and size for Log mode */
				u16_return_val = SetLogData( &u08_RM_RcvBufferArray[RM_FRAME_PAYLOAD], u08_payloadsize );

				u08_RM_CommMode = RM_CANDR_MODE;

				break;

			case 0x06:
				/* Request Condition */
				u16_return_val = RequestDUTCondition( &u08_RM_RcvBufferArray[RM_FRAME_PAYLOAD], u08_payloadsize );

				u08_RM_CommMode = RM_CANDR_MODE;

				break;

			case 0x07:
				/* Request Memory Dump */
				u16_return_val = SetDumpData( &u08_RM_RcvBufferArray[RM_FRAME_PAYLOAD], u08_payloadsize );

				u08_RM_CommMode = RM_CANDR_MODE;

				break;

			default:

				break;
			}

		}


		if( u16_return_val != RM_STATUS_ERR )
		{
			u08_RM_MasCnt = u08_mas_cnt;

			u08_trg = RM_TRUE;

		}

		u08_RM_RcvBufferSize = 0;

	}


	if( u08_RM_DelayedTrg == RM_TRUE )
	{
		u08_RM_DelayedTrg = RM_FALSE;
		u08_trg = RM_TRUE;

	}
	else
	{
		u08_trg = CtrlSendTiming( u08_RM_CommMode, u08_trg );

	}


	if( u08_RM_SndSLIPActiveFlg == RM_FALSE )
	{
		u08_send_flg = RM_FALSE;

		if( u08_trg == RM_TRUE )
		{
			if( u08_RM_CommMode == RM_LOG_MODE )
			{
				u16_return_val = GetLogData( &u08_RM_SndBufferArray[RM_FRAME_PAYLOAD], &u08_CommPayloadSize );

			}
			else
			{
				u16_return_val = GetBlockData( &u08_RM_SndBufferArray[RM_FRAME_PAYLOAD], &u08_CommPayloadSize );

			}

			if( u16_return_val != RM_STATUS_ERR )
			{
				u08_send_flg = RM_TRUE;

			}

		}

		if( u08_send_flg == RM_TRUE )
		{
			/* response opcode */
			u08_RM_SndBufferArray[RM_FRAME_SEQCODE] = u08_RM_MasCnt + u08_RM_SlvCnt;

			u08_RM_SlvCnt++;

			if( u08_RM_SlvCnt > 0x0F )
			{
				u08_RM_SlvCnt = 0x01;

			}

			u08_RM_SndBufferSize = 1;

			/* payload(data) */
			/* Data is already stored in GetLogData() or GetBlockData() */
			u08_RM_SndBufferSize += u08_CommPayloadSize;

			/* payload(crc) */
			u08_crc = GetCRC(u08_RM_SndBufferArray, u08_RM_SndBufferSize);
			u08_RM_SndBufferArray[u08_RM_SndBufferSize] = u08_crc;
			u08_RM_SndBufferSize++;

			u08_RM_SndBufferSentIndex = 0;

		}

	}
	else
	{
		if( u08_trg == RM_TRUE )
		{
			u08_RM_DelayedTrg = RM_TRUE;

		}

	}

}


/**
  * @brief  Configure interval transmitting timming.
  * @param  u08_mode "RM state"
  * @param  u08_trg transmitting request
  * @retval assert transmitting
  */
static	uint8_t		CtrlSendTiming( uint8_t u08_mode, uint8_t u08_trg )
{
	uint8_t		u08_send_flg;

	u08_send_flg = RM_FALSE;

	if( u08_mode != RM_LOG_MODE )
	{
		if( u08_trg == RM_TRUE )
		{
			u08_send_flg = RM_TRUE;

		}

		u16_RM_ReqTimeoutCnt = 0;
		u16_RM_WaitCnt = RM_BASE_CYCLE;

	}
	else
	{
		if( u08_trg == RM_TRUE )
		{
			u16_RM_ReqTimeoutCnt = 0;

		}

		if( u16_RM_ReqTimeoutCnt >= RM_REQ_TIMEOUT_CNT )
		{
			/* Stop Interval transmission */
			u16_RM_ReqTimeoutCnt = RM_REQ_TIMEOUT_CNT;

			u16_RM_TimingCnt = RM_SND_INTERVAL_CNT;

			u16_RM_WaitCnt = RM_BASE_CYCLE;

		}
		else
		{
			/* Continue Interval transmission */
			u16_RM_ReqTimeoutCnt = u16_RM_ReqTimeoutCnt + RM_BASE_CYCLE;

			if( u16_RM_TimingCnt == 0 )
			{
				if( u08_trg == RM_TRUE )
				{
					u08_send_flg = RM_TRUE;

				}

				u16_RM_WaitCnt = RM_BASE_CYCLE;

			}
			else if( u16_RM_WaitCnt >= u16_RM_TimingCnt )
			{
				u16_RM_WaitCnt = RM_BASE_CYCLE;

				u08_send_flg = RM_TRUE;

			}
			else
			{
				u16_RM_WaitCnt = u16_RM_WaitCnt + RM_BASE_CYCLE;

			}

		}


	}

	return( u08_send_flg );

}


/**
  * @brief  Start interval data transmission
  * @param  u08_tmp[] Frame payload array
  * @param  u08_payloadsize Frame payload size
  * @retval status
  */
static	uint16_t	SetLogStart( uint8_t u08_tmp[], uint8_t u08_payloadsize )
{
	uint16_t	u16_return_val;

	u16_return_val = RM_STATUS_SUCCESS;

	if( u08_payloadsize != 0 )
	{
		u16_return_val = RM_STATUS_ERR;

	}

	/* Return payload(data) size is zero */
	u08_RM_DumpLentgh = 0;

	return( u16_return_val );

}


/**
  * @brief  Stop interval data transmission
  * @param  u08_tmp[] Frame payload array
  * @param  u08_payloadsize Frame payload size
  * @retval status
  */
static	uint16_t	SetLogStop( uint8_t u08_tmp[], uint8_t u08_payloadsize )
{
	uint16_t	u16_return_val;

	u16_return_val = RM_STATUS_SUCCESS;

	if( u08_payloadsize != 0 )
	{
		u16_return_val = RM_STATUS_ERR;

	}

	/* Return payload(data) size is zero */
	u08_RM_DumpLentgh = 0;

	return( u16_return_val );

}


/**
  * @brief  Configure interval data transmission timming
  * @param  u08_tmp[] Frame payload array
  * @param  u08_payloadsize Frame payload size
  * @retval status
  */
static	uint16_t	SetTiming( uint8_t u08_tmp[], uint8_t u08_payloadsize )
{
	uint16_t	u16_timing;

	uint16_t	u16_return_val;

	u16_return_val = RM_STATUS_SUCCESS;

	if( u08_payloadsize != 2 )
	{
		u16_return_val = RM_STATUS_ERR;

	}
	else
	{
		u16_timing  = (uint16_t)u08_tmp[1];
		u16_timing  = u16_timing << 8;
		u16_timing |= u08_tmp[0];

		u16_RM_TimingCnt = u16_timing;

	}

	/* Return payload(data) size is zero */
	u08_RM_DumpLentgh = 0;

	return( u16_return_val );

}


/**
  * @brief  Write data to specified RAM Address
  * @param  u08_tmp[] Frame payload array
  * @param  u08_payloadsize Frame payload size
  * @retval status
  */
static	uint16_t	WriteData( uint8_t u08_tmp[], uint8_t u08_payloadsize )
{
#ifdef	RM_ADDRESS_4BYTE
#define	TABLE_SIZE			9
static	const	uint8_t		u08_RM_ValidWrValTable[TABLE_SIZE] =
{
	0,	0,	0,	0,	0,			/* size(1) and address(4) */
	1,	2,	0,	4				/* data length */
};

	uint32_t	var_address;

#else
#define	TABLE_SIZE			7
static	const	uint8_t		u08_RM_ValidWrValTable[TABLE_SIZE] =
{
	0,	0,	0,					/* size(1) and address(2) */
	1,	2,	0,	4				/* data length */
};

	uint16_t	var_address;

#endif

	uint16_t	u16_index;

	uint8_t		u08_size;

	uint8_t		u08_buff;
	uint16_t	u16_buff;
	uint32_t	u32_buff;

	uint8_t*	pu08_addr;
	uint16_t*	pu16_addr;
	uint32_t*	pu32_addr;

	uint16_t	u16_return_val;

	u16_return_val = RM_STATUS_SUCCESS;

	if( (u08_payloadsize > TABLE_SIZE) ||
		(u08_payloadsize == 0 ) )
	{
		u16_return_val = RM_STATUS_ERR;

	}
	else
	{
		u08_size = u08_tmp[0];

		if( (u08_size != 0) &&
			(u08_size == u08_RM_ValidWrValTable[(u08_payloadsize - 1)]) )
		{
			/* u08_tmp(recieved data) is stored as LSB first  */
#ifdef	RM_ADDRESS_4BYTE
			var_address  = (uint32_t)u08_tmp[4];
			var_address  = var_address << 8;
			var_address |= (uint32_t)u08_tmp[3];
			var_address  = var_address << 8;
			var_address |= (uint32_t)u08_tmp[2];
			var_address  = var_address << 8;
			var_address |= (uint32_t)u08_tmp[1];

			/* set next index */
			u16_index = 5;

#else
			var_address  = (uint16_t)u08_tmp[2];
			var_address  = var_address << 8;
			var_address |= (uint16_t)u08_tmp[1];

			/* set next index */
			u16_index = 3;

#endif

			switch( u08_size )
			{
			case 1:
				u08_buff = u08_tmp[u16_index];

				pu08_addr = (uint8_t *)var_address;
				*pu08_addr = u08_buff;

				break;

			case 2:
				/* u08_tmp(recieved data) is stored as LSB first  */
				u16_buff  = (uint16_t)u08_tmp[u16_index+1];
				u16_buff  = u16_buff << 8;
				u16_buff |= (uint16_t)u08_tmp[u16_index];

				pu16_addr = (uint16_t *)var_address;
				*pu16_addr = u16_buff;

				break;

			case 4:
				/* u08_tmp(recieved data) is stored as LSB first  */
				u32_buff  = (uint32_t)u08_tmp[u16_index+3];
				u32_buff  = u32_buff << 8;
				u32_buff |= (uint32_t)u08_tmp[u16_index+2];
				u32_buff  = u32_buff << 8;
				u32_buff |= (uint32_t)u08_tmp[u16_index+1];
				u32_buff  = u32_buff << 8;
				u32_buff |= (uint32_t)u08_tmp[u16_index];

				pu32_addr = (uint32_t *)var_address;
				*pu32_addr = u32_buff;

				break;

			default:
				u16_return_val = RM_STATUS_ERR;

				break;

			}

		}

	}

	/* Return payload(data) size is zero */
	u08_RM_DumpLentgh = 0;

	return( u16_return_val );

#undef	TABLE_SIZE

}


/**
  * @brief  Set Address data and Size data in using LOG_MODE.
  * @param  u08_tmp[] Frame payload array
  * @param  u08_payloadsize Frame payload size
  * @retval status
  */
static	uint16_t	SetLogData( uint8_t u08_tmp[], uint8_t u08_payloadsize )
{
static	uint8_t		u08_ReceptionSize;
static	uint8_t		u08_SetLogErrFlg;
static	uint8_t		u08_LogBaseIndex;

#ifdef	RM_ADDRESS_4BYTE
#define	TABLE_SIZE			21
#define	MAX_LOGBASE_INDEX	7
#define	LOG_INDEX_OFFSET	4
#define	DATA_UNIT_NUM		5	/* size(1)+address(4) = 5 */
static	const	uint8_t		u08_RM_ValidLengthTable[TABLE_SIZE] =
{
	0,										/* code */
	0,	0,	0,	0,	1,	0,	0,	0,	0,	2,	/* (size(1) and address(4)) * 2 */
	0,	0,	0,	0,	3,	0,	0,	0,	0,	4	/* (size(1) and address(4)) * 2 */
};

	uint32_t	var_address;

#else
#define	TABLE_SIZE			25
#define	MAX_LOGBASE_INDEX	3
#define	LOG_INDEX_OFFSET	8
#define	DATA_UNIT_NUM		3	/* size(1)+address(2) = 3 */
static	const	uint8_t		u08_RM_ValidLengthTable[TABLE_SIZE] =
{
	0,						/* code */
	0,	0,	1,	0,	0,	2,	/* (size(1) and address(2)) * 2 */
	0,	0,	3,	0,	0,	4,	/* (size(1) and address(2)) * 2 */
	0,	0,	5,	0,	0,	6,	/* (size(1) and address(2)) * 2 */
	0,	0,	7,	0,	0,	8	/* (size(1) and address(2)) * 2 */
};

	uint16_t	var_address;

#endif
	uint16_t	u16_base_index;
	uint16_t	u16_log_index;

	uint16_t	u16_index;

	uint8_t		u08_state_flg;

	uint8_t		u08_max_index;

	uint16_t	u16_return_val;

	u16_return_val = RM_STATUS_SUCCESS;

	if( (u08_payloadsize > TABLE_SIZE) ||
		(u08_payloadsize == 0 ) )
	{
		u08_SetLogErrFlg = RM_TRUE;

		u16_return_val = RM_STATUS_ERR;

	}
	else
	{
		u08_max_index = u08_RM_ValidLengthTable[(u08_payloadsize - 1)];

		if( u08_max_index == 0 )
		{
			u08_SetLogErrFlg = RM_TRUE;

			u16_return_val = RM_STATUS_ERR;

		}
		else
		{
			u08_state_flg = u08_tmp[0] & SETLOG_BIT_MASK;

			/* begin */
			if( (u08_state_flg & SETLOG_START_BIT ) == SETLOG_START_BIT )
			{
				u08_RM_ValidSize = 0;

				u08_ReceptionSize = 0;
				u08_SetLogErrFlg = RM_FALSE;
				u08_LogBaseIndex = 0;

			}


			if( (u08_RM_ValidSize == 0 ) &&
				(u08_SetLogErrFlg != RM_TRUE ) )
			{
				for( u16_index = 0; u16_index < (uint16_t)u08_max_index; u16_index++ )
				{
					if( u08_LogBaseIndex > MAX_LOGBASE_INDEX )
					{
						u08_LogBaseIndex = MAX_LOGBASE_INDEX;

						u08_SetLogErrFlg = RM_TRUE;

						u16_return_val = RM_STATUS_ERR;
						break;

					}
					else
					{
						u16_log_index = (uint16_t)u08_LogBaseIndex;
						u16_log_index = u16_log_index * LOG_INDEX_OFFSET;

					}

					u16_base_index = u16_index * DATA_UNIT_NUM;

#ifdef	RM_ADDRESS_4BYTE
					/* u08_tmp(recieved data) is stored as LSB first  */
					var_address  = (uint32_t)u08_tmp[1+u16_base_index+4];
					var_address  = var_address << 8;
					var_address |= (uint32_t)u08_tmp[1+u16_base_index+3];
					var_address  = var_address << 8;
					var_address |= (uint32_t)u08_tmp[1+u16_base_index+2];
					var_address  = var_address << 8;
					var_address |= (uint32_t)u08_tmp[1+u16_base_index+1];

					u08_RM_SizeArray[u16_log_index+u16_index] = u08_tmp[1+u16_base_index];
					u32_RM_AddressArray[u16_log_index+u16_index] = var_address;

#else
					/* u08_tmp(recieved data) is stored as LSB first  */
					var_address  = (uint16_t)u08_tmp[1+u16_base_index+2];
					var_address  = var_address << 8;
					var_address |= (uint16_t)u08_tmp[1+u16_base_index+1];

					u08_RM_SizeArray[u16_log_index+u16_index] = u08_tmp[1+u16_base_index];
					u16_RM_AddressArray[u16_log_index+u16_index] = var_address;

#endif
					u08_ReceptionSize++;

				}

			}

			u08_LogBaseIndex++;

			/* end */
			if( ( (u08_state_flg & SETLOG_END_BIT ) == SETLOG_END_BIT ) &&
				  (u08_SetLogErrFlg != RM_TRUE ) )
			{
				u08_RM_ValidSize = u08_ReceptionSize;

			}

		}

	}

	if( u16_return_val == RM_STATUS_ERR )
	{
		/* Invalid */
		u08_RM_ValidSize = 0;

	}

	/* Return payload(data) size is zero */
	u08_RM_DumpLentgh = 0;

	return( u16_return_val );

#undef	TABLE_SIZE
#undef	MAX_LOGBASE_INDEX
#undef	LOG_INDEX_OFFSET
#undef	DATA_UNIT_NUM

}


/**
  * @brief  To get DUT Condition(Ex. Version strings) with checking Password.
  * @param  u08_tmp[] Frame payload array
  * @param  u08_payloadsize Frame payload size
  * @retval status
  */
static	uint16_t	RequestDUTCondition( uint8_t u08_tmp[], uint8_t u08_payloadsize )
{
	uint32_t	u32_password;
	uint16_t	u16_return_val;

	u16_return_val = RM_STATUS_SUCCESS;

	if( u08_payloadsize != 4 )
	{
		u16_return_val = RM_STATUS_ERR;

	}
	else
	{
		u32_password  = u08_tmp[3];
		u32_password  = u32_password << 8;
		u32_password |= u08_tmp[2];
		u32_password  = u32_password << 8;
		u32_password |= u08_tmp[1];
		u32_password  = u32_password << 8;
		u32_password |= u08_tmp[0];

		if( u32_password != RM_PASSWORD )
		{
			u16_return_val = RM_STATUS_ERR;

		}
		else
		{
#ifdef	RM_ADDRESS_4BYTE
			u32_RM_DumpAddress = u32_RM_VersionAddress;
#else
			u16_RM_DumpAddress = u16_RM_VersionAddress;
#endif

			u08_RM_DumpLentgh = u08_RM_VersionLength;

		}

	}

	return( u16_return_val );

}


/**
  * @brief  To get specified data length from specified RAM Address
  * @param  u08_tmp[] Frame payload array
  * @param  u08_payloadsize Frame payload size
  * @retval status
  */
static	uint16_t	SetDumpData( uint8_t u08_tmp[], uint8_t u08_payloadsize )
{
#ifdef	RM_ADDRESS_4BYTE
#define	RECEPTION_SIZE 5

	uint32_t	var_address;
#else
#define	RECEPTION_SIZE 3

	uint16_t	var_address;
#endif

	uint16_t	u16_return_val;

	u16_return_val = RM_STATUS_SUCCESS;

	if( u08_payloadsize != RECEPTION_SIZE )
	{
		u16_return_val = RM_STATUS_ERR;

	}
	else
	{
#ifdef	RM_ADDRESS_4BYTE
		u08_RM_DumpLentgh = u08_tmp[4];

		var_address  = (uint32_t)u08_tmp[3];
		var_address  = var_address << 8;
		var_address |= (uint32_t)u08_tmp[2];
		var_address  = var_address << 8;
		var_address |= (uint32_t)u08_tmp[1];
		var_address  = var_address << 8;
		var_address |= u08_tmp[0];

		u32_RM_DumpAddress = var_address;

#else
		u08_RM_DumpLentgh = u08_tmp[2];

		var_address  = (uint16_t)u08_tmp[1];
		var_address  = var_address << 8;
		var_address |= u08_tmp[0];

		u16_RM_DumpAddress = var_address;

#endif

	}

	return( u16_return_val );

#undef	RECEPTION_SIZE

}


/**
  * @brief  store a payload from specified address and size based on "SetLogData" function.
  * @param  u08_tmp[] Frame payload array
  * @param  u08_payloadsize Frame payload size
  * @retval status
  */
static	uint16_t	GetLogData( uint8_t u08_tmp[], uint8_t *u08_frame_size )
{
#ifdef	RM_ADDRESS_4BYTE
	uint32_t	var_address;
#else
	uint16_t	var_address;
#endif
	uint16_t	u16_index;
	uint16_t	u16_send_index;

	uint16_t	u16_total_size;

	uint8_t		u08_buff;
	uint16_t	u16_buff;
	uint32_t	u32_buff;

	uint8_t*	pu08_addr;
	uint16_t*	pu16_addr;
	uint32_t*	pu32_addr;

	uint16_t	u16_return_val;

	u16_send_index = 0;
	u16_return_val = RM_STATUS_SUCCESS;

	u16_total_size = 0;

	for( u16_index = 0; u16_index < u08_RM_ValidSize; u16_index++ )
	{
		u16_total_size += (uint16_t)u08_RM_SizeArray[u16_index];
	}

	if( u16_total_size >= RM_SND_PAYLOAD_SIZE )
	{
		return( RM_STATUS_ERR );

	}

	for( u16_index = 0; u16_index < u08_RM_ValidSize; u16_index++ )
	{
#ifdef	RM_ADDRESS_4BYTE
		var_address = u32_RM_AddressArray[u16_index];
#else
		var_address = u16_RM_AddressArray[u16_index];
#endif

		switch( u08_RM_SizeArray[u16_index] )
		{
		case 1:
			pu08_addr = (uint8_t *)var_address;

			u08_buff = *pu08_addr;

			u08_tmp[u16_send_index] = u08_buff;
			u16_send_index++;

			break;

		case 2:
			pu16_addr = (uint16_t *)var_address;

			u16_buff = *pu16_addr;

			/* u08_tmp(data will be sent) is stored as LSB first  */
			u08_tmp[u16_send_index] = (uint8_t)(u16_buff);
			u16_send_index++;
			u16_buff = u16_buff >> 8;
			u08_tmp[u16_send_index] = (uint8_t)(u16_buff);
			u16_send_index++;

			break;

		case 4:
			pu32_addr = (uint32_t *)var_address;

			u32_buff = *pu32_addr;

			/* u08_tmp(data will be sent) is stored as LSB first  */
			u08_tmp[u16_send_index] = (uint8_t)(u32_buff);
			u16_send_index++;
			u32_buff = u32_buff >> 8;
			u08_tmp[u16_send_index] = (uint8_t)(u32_buff);
			u16_send_index++;
			u32_buff = u32_buff >> 8;
			u08_tmp[u16_send_index] = (uint8_t)(u32_buff);
			u16_send_index++;
			u32_buff = u32_buff >> 8;
			u08_tmp[u16_send_index] = (uint8_t)(u32_buff);
			u16_send_index++;

			break;

		default:
			u16_return_val = RM_STATUS_ERR;

			break;

		}

		if( u16_return_val == RM_STATUS_ERR )
		{
			break;

		}

	}

	if( u16_return_val == RM_STATUS_SUCCESS )
	{
		*u08_frame_size = (uint8_t)u16_send_index;

	}

	return( u16_return_val );

}


/**
  * @brief  store a payload from specified address and length
  * @param  u08_tmp[] Frame payload array
  * @param  u08_payloadsize Frame payload size
  * @retval status
  */
static	uint16_t	GetBlockData( uint8_t u08_tmp[], uint8_t *u08_frame_size )
{
#ifdef	RM_ADDRESS_4BYTE
	uint32_t	var_address;
#else
	uint16_t	var_address;
#endif
	uint16_t	u16_index;
	uint16_t	u16_send_index;

	uint8_t		u08_buff;
	uint8_t*	pu08_addr;

	uint16_t	u16_return_val;

#ifdef	RM_ADDRESS_4BYTE
	var_address = u32_RM_DumpAddress;
#else
	var_address = u16_RM_DumpAddress;
#endif

	u16_send_index = 0;
	u16_return_val = RM_STATUS_SUCCESS;

	pu08_addr = (uint8_t *)var_address;

	for( u16_index = 0; u16_index < (uint8_t)u08_RM_DumpLentgh; u16_index++ )
	{
		u08_buff = *pu08_addr;

		if( u16_send_index >= RM_SND_PAYLOAD_SIZE )
		{
			u16_return_val = RM_STATUS_ERR;

		}
		else
		{
			u08_tmp[u16_send_index] = u08_buff;
			u16_send_index++;
			pu08_addr++;

		}

	}

	*u08_frame_size = (uint8_t)u16_send_index;

	return( u16_return_val );

}


/**
  * @brief  calculate CRC8 from byte buffer.
  * @param  u08_buff[] buffer array
  * @param  u08_buff_size buffer size
  * @retval status
  */
static	uint8_t		GetCRC( uint8_t u08_buff[], uint8_t u08_buff_size )
{
/* x^8+x^7+x^4+x^2+1 */
static	const	uint8_t		u08_RM_CrcTable[256] =
{
 /*      0     1     2     3     4     5     6     7     8     9     A     B     C     D     E     F         */
	  0x00, 0xd5, 0x7f, 0xaa, 0xfe, 0x2b, 0x81, 0x54, 0x29, 0xfc, 0x56, 0x83, 0xd7, 0x02, 0xa8, 0x7d /* 0x00 */
	, 0x52, 0x87, 0x2d, 0xf8, 0xac, 0x79, 0xd3, 0x06, 0x7b, 0xae, 0x04, 0xd1, 0x85, 0x50, 0xfa, 0x2f /* 0x10 */
	, 0xa4, 0x71, 0xdb, 0x0e, 0x5a, 0x8f, 0x25, 0xf0, 0x8d, 0x58, 0xf2, 0x27, 0x73, 0xa6, 0x0c, 0xd9 /* 0x20 */
	, 0xf6, 0x23, 0x89, 0x5c, 0x08, 0xdd, 0x77, 0xa2, 0xdf, 0x0a, 0xa0, 0x75, 0x21, 0xf4, 0x5e, 0x8b /* 0x30 */
	, 0x9d, 0x48, 0xe2, 0x37, 0x63, 0xb6, 0x1c, 0xc9, 0xb4, 0x61, 0xcb, 0x1e, 0x4a, 0x9f, 0x35, 0xe0 /* 0x40 */
	, 0xcf, 0x1a, 0xb0, 0x65, 0x31, 0xe4, 0x4e, 0x9b, 0xe6, 0x33, 0x99, 0x4c, 0x18, 0xcd, 0x67, 0xb2 /* 0x50 */
	, 0x39, 0xec, 0x46, 0x93, 0xc7, 0x12, 0xb8, 0x6d, 0x10, 0xc5, 0x6f, 0xba, 0xee, 0x3b, 0x91, 0x44 /* 0x60 */
	, 0x6b, 0xbe, 0x14, 0xc1, 0x95, 0x40, 0xea, 0x3f, 0x42, 0x97, 0x3d, 0xe8, 0xbc, 0x69, 0xc3, 0x16 /* 0x70 */
	, 0xef, 0x3a, 0x90, 0x45, 0x11, 0xc4, 0x6e, 0xbb, 0xc6, 0x13, 0xb9, 0x6c, 0x38, 0xed, 0x47, 0x92 /* 0x80 */
	, 0xbd, 0x68, 0xc2, 0x17, 0x43, 0x96, 0x3c, 0xe9, 0x94, 0x41, 0xeb, 0x3e, 0x6a, 0xbf, 0x15, 0xc0 /* 0x90 */
	, 0x4b, 0x9e, 0x34, 0xe1, 0xb5, 0x60, 0xca, 0x1f, 0x62, 0xb7, 0x1d, 0xc8, 0x9c, 0x49, 0xe3, 0x36 /* 0xA0 */
	, 0x19, 0xcc, 0x66, 0xb3, 0xe7, 0x32, 0x98, 0x4d, 0x30, 0xe5, 0x4f, 0x9a, 0xce, 0x1b, 0xb1, 0x64 /* 0xB0 */
	, 0x72, 0xa7, 0x0d, 0xd8, 0x8c, 0x59, 0xf3, 0x26, 0x5b, 0x8e, 0x24, 0xf1, 0xa5, 0x70, 0xda, 0x0f /* 0xC0 */
	, 0x20, 0xf5, 0x5f, 0x8a, 0xde, 0x0b, 0xa1, 0x74, 0x09, 0xdc, 0x76, 0xa3, 0xf7, 0x22, 0x88, 0x5d /* 0xD0 */
	, 0xd6, 0x03, 0xa9, 0x7c, 0x28, 0xfd, 0x57, 0x82, 0xff, 0x2a, 0x80, 0x55, 0x01, 0xd4, 0x7e, 0xab /* 0xE0 */
	, 0x84, 0x51, 0xfb, 0x2e, 0x7a, 0xaf, 0x05, 0xd0, 0xad, 0x78, 0xd2, 0x07, 0x53, 0x86, 0x2c, 0xf9 /* 0xF0 */
};

	uint8_t		u08_index;

	uint8_t		u08_crc_index;

	uint8_t		u08_crc;

	u08_crc = 0;

	for( u08_index = 0; u08_index < u08_buff_size; u08_index++ )
	{
		u08_crc_index = u08_crc ^ u08_buff[ u08_index ];
		u08_crc = u08_RM_CrcTable[ u08_crc_index ];
	}

	return( u08_crc );

}


/**
  * @brief  check
  * @param  u08_buff[] buffer array
  * @param  u08_buff_size buffer size
  * @retval status
  */
#ifdef	RM_ADDRESS_4BYTE
static	uint16_t	CheckSizeAndAddress( uint8_t u08_size, uint32_t var_address )
#else
static	uint16_t	CheckSizeAndAddress( uint8_t u08_size, uint16_t var_address )
#endif
{
	uint16_t	u16_return_val;

	u16_return_val = RM_STATUS_SUCCESS;

	if( (u08_size != 1) &&
		(u08_size != 2) &&
		(u08_size != 4) )
	{
		u16_return_val = RM_STATUS_ERR;

	}
	else if( (u08_size != 1) &&
			((var_address & 0x01) == 0x01) )
	{
#ifndef	RM_ADDRESS_1BYTE
		u16_return_val = RM_STATUS_ERR;
#endif

	}
	else
	{

	}

	return( u16_return_val );

}


void	RM_Initial( uint8_t u08_tmp[], uint8_t u08_buff_size )
{
#ifdef	RM_ADDRESS_4BYTE
	u32_RM_VersionAddress = (uint32_t)u08_tmp;
#else
	u16_RM_VersionAddress = (uint16_t)u08_tmp;
#endif
	u08_RM_VersionLength = u08_buff_size;

	u08_RM_ValidSize = 0;

	u16_RM_RcvTimeoutCnt = 0;
	u16_RM_ReqTimeoutCnt = 0;
	u16_RM_WaitCnt = RM_BASE_CYCLE;
	u16_RM_TimingCnt = RM_SND_INTERVAL_CNT;

	u08_RM_RcvBufferSize = 0;
	u08_RM_SndBufferSize = 0;

	u08_RM_SndBufferSentIndex = 0;

	u08_RM_MasCnt = 0x00;
	u08_RM_SlvCnt = 0x00;	/* initial slv_cnt is "0xX0" */

	u08_RM_CommMode = RM_INITIAL_MODE;

	u08_RM_RcvRingBuffEndIndex = 0;
	u08_RM_RcvRingBuffBgnIndex = 0;
	u08_RM_RcvSLIPActiveFlg = RM_FALSE;

	u08_RM_SndSLIPActiveFlg = RM_FALSE;

	u08_RM_ValidFlg = RM_FALSE;

	u08_RM_DelayedTrg = RM_FALSE;

}


uint8_t*	RM_GetSendSoFrame( void )
{
	uint8_t*	pu08_char;

	if( u08_RM_SndSLIPActiveFlg == RM_FALSE )
	{
		pu08_char = RM_GetSendFrame();

	}
	else
	{
		pu08_char = RM_RET_NULL;

	}

	return( pu08_char );

}


uint8_t*	RM_GetSendFrame( void )
{
static	uint8_t		u08_SndSLIPTmpData;
static	uint8_t		u08_SndSLIPeofFlg;
static	uint8_t		u08_SndSLIPEscChar;
static	uint8_t		u08_SndSLIPLastChar;

	uint8_t*	pu08_char;
	uint8_t		u08_size;

	pu08_char = RM_RET_NULL;

	if( u08_RM_SndBufferSentIndex < u08_RM_SndBufferSize )
	{
		if( u08_RM_SndSLIPActiveFlg == RM_FALSE )
		{
			u08_SndSLIPLastChar = 0x00;
			u08_SndSLIPeofFlg = RM_FALSE;
			u08_RM_SndSLIPActiveFlg = RM_TRUE;

			u08_SndSLIPTmpData = RM_FRAME_CHAR_END;
			pu08_char = &u08_SndSLIPTmpData;

		}
		else
		{
			if( u08_SndSLIPLastChar == RM_FRAME_CHAR_ESC )
			{
				u08_SndSLIPTmpData = u08_SndSLIPEscChar;

			}
			else
			{
				u08_SndSLIPTmpData = u08_RM_SndBufferArray[ u08_RM_SndBufferSentIndex ];

			}

			if( u08_SndSLIPTmpData == RM_FRAME_CHAR_END )
			{
				u08_SndSLIPTmpData = RM_FRAME_CHAR_ESC;
				u08_SndSLIPEscChar = RM_FRAME_CHAR_ESC_END;

			}
			else if( u08_SndSLIPTmpData == RM_FRAME_CHAR_ESC )
			{
				u08_SndSLIPTmpData = RM_FRAME_CHAR_ESC;
				u08_SndSLIPEscChar = RM_FRAME_CHAR_ESC_ESC;

			}
			else
			{
				u08_RM_SndBufferSentIndex++;

			}

			u08_SndSLIPLastChar = u08_SndSLIPTmpData;
			pu08_char = &u08_SndSLIPTmpData;

		}

	}
	else
	{
		if( u08_RM_SndSLIPActiveFlg == RM_TRUE )
		{
			if( u08_SndSLIPeofFlg == RM_FALSE )
			{
				u08_SndSLIPeofFlg = RM_TRUE;
				u08_SndSLIPTmpData = RM_FRAME_CHAR_END;
				pu08_char = &u08_SndSLIPTmpData;

			}
			else
			{
				u08_SndSLIPeofFlg = RM_FALSE;
				u08_RM_SndSLIPActiveFlg = RM_FALSE;

			}

		}

	}

	return( pu08_char );

}


uint16_t	RM_PutReceiveBuff( uint8_t u08_char )
{
	uint16_t	u16_return_val;

	u16_return_val = RM_BUFFER_FULL;

	if( ((u08_RM_RcvRingBuffEndIndex + 1) & RM_UART_BUFF_MASK ) != u08_RM_RcvRingBuffBgnIndex )
	{
		u08_RM_RcvRingBuffArray[ u08_RM_RcvRingBuffEndIndex ] = u08_char;
		u08_RM_RcvRingBuffEndIndex = (u08_RM_RcvRingBuffEndIndex + 1) & RM_UART_BUFF_MASK;

		u16_return_val = RM_BUFFER_OK;
	}

	return( u16_return_val );
}


static	uint16_t	GetReceiveFrame( uint8_t u08_tmp[], uint8_t *u08_rcv_index )
{
static	uint8_t		u08_RcvSLIPLastChar;

	uint8_t		u08_val;
	uint8_t		u08_size;
	uint8_t		u08_crc;

	uint16_t	u16_err_occur;

	uint16_t	u16_return_val;


	if( u08_RM_RcvSLIPActiveFlg == RM_FALSE )
	{
		u16_return_val = RM_SLIP_RCV_NOTHING;

	}
	else
	{
		u16_return_val = RM_SLIP_RCV_INCOMING;

	}


	u16_err_occur = 0;

	u08_size = (u08_RM_RcvRingBuffEndIndex - u08_RM_RcvRingBuffBgnIndex) & RM_UART_BUFF_MASK;

	while(u08_size != 0)
	{
		u08_val = u08_RM_RcvRingBuffArray[ u08_RM_RcvRingBuffBgnIndex ];
		u08_RM_RcvRingBuffBgnIndex = (u08_RM_RcvRingBuffBgnIndex + 1) & RM_UART_BUFF_MASK;

		u08_size--;

		if( u08_val == RM_FRAME_CHAR_END )
		{

			if( u08_RM_RcvSLIPActiveFlg == RM_FALSE )
			{
				/* Start SLIP Frame */
				u08_RM_RcvSLIPActiveFlg = RM_TRUE;

				u16_return_val = RM_SLIP_RCV_INCOMING;

				u08_RcvSLIPLastChar = 0x00;

			}
			else
			{
				if( *u08_rcv_index == 0 )
				{
					/* receive order RM_FRAME_CHAR_END,RM_FRAME_CHAR_END */
					/* Last RM_FRAME_CHAR_END might be Start of Frame */

				}
				else
				{
					u08_crc = GetCRC(u08_tmp, *u08_rcv_index);

					if( u08_crc != 0 )
					{
						*u08_rcv_index = 0;

					}
					else
					{
						/* End SLIP Frame */
						u08_RM_RcvSLIPActiveFlg = RM_FALSE;

						u16_return_val = RM_SLIP_RCV_COMPLETE;

						/* delete useless crc data size */
						(*u08_rcv_index)--;

						break;

					}

				}

			}

		}
		else
		{
			if( u08_RM_RcvSLIPActiveFlg == RM_FALSE )
			{
				/* Ignore receive data */

			}
			else
			{
				/* payload of frame */
				u16_return_val = RM_SLIP_RCV_INCOMING;

				if( u08_RcvSLIPLastChar == RM_FRAME_CHAR_ESC )
				{
					if( u08_val == RM_FRAME_CHAR_ESC_END )
					{
						u08_tmp[*u08_rcv_index] = RM_FRAME_CHAR_END;
						(*u08_rcv_index)++;

					}
					else if( u08_val == RM_FRAME_CHAR_ESC_ESC )
					{
						u08_tmp[*u08_rcv_index] = RM_FRAME_CHAR_ESC;
						(*u08_rcv_index)++;

					}
					else
					{
						u16_err_occur = 1;

					}
				}
				else
				{
					if( u08_val == RM_FRAME_CHAR_ESC )
					{
						/* Start escape sequence */

					}
					else
					{
						u08_tmp[*u08_rcv_index] = u08_val;
						(*u08_rcv_index)++;

					}

				}

				if( *u08_rcv_index >= RM_RCV_FRAME_BUFF_SIZE )
				{
					u16_err_occur = 2;

				}

			}


		}

		u08_RcvSLIPLastChar = u08_val;

		if( u16_err_occur != 0 )
		{
			u08_RM_RcvSLIPActiveFlg = RM_FALSE;

			*u08_rcv_index = 0;

			u16_return_val = RM_SLIP_RCV_FAULT;

			break;

		}

	}

	return( u16_return_val );

}


/*SS
void	RM_isLogMode( void )
EE*/
static	uint8_t		RM_isLogMode( void )
{
	if( u08_RM_CommMode == RM_LOG_MODE )
	{
		return( RM_TRUE );

	}
	else
	{
		return( RM_FALSE );

	}

}


/*-- end of file --*/
