#ifndef		RM_COMM_H
#define		RM_COMM_H

#ifdef __cplusplus
 extern "C" {
#endif

#if 1
#include "stdint.h"

#else
typedef	  signed	char	 int8_t;
typedef	unsigned	char	uint8_t;
typedef	  signed	short	 int16_t;
typedef	unsigned	short	uint16_t;
typedef	  signed	long	 int32_t;
typedef	unsigned	long	uint32_t;


#endif

#define	RM_ADDRESS_2BYTE

#define	RM_PASSWORD				0x0000FFFFU

#define	RM_BASE_CYCLE			5			/* Call RM_Communiation() timing */


#define	RM_RET_NULL				(uint8_t *)0x00000000

#define	RM_BUFFER_OK			1
#define	RM_BUFFER_FULL			0

#define	RM_TRUE					1
#define	RM_FALSE				0


/*-- begin:	functions --*/
void		RM_Communiation( void );
void		RM_Initial( uint8_t u08_tmp[], uint8_t u08_buff_size );

uint16_t	RM_PutReceiveBuff( uint8_t u08_char );

uint8_t*	RM_GetSendSoFrame( void );
uint8_t*	RM_GetSendFrame( void );

#ifdef __cplusplus
}
#endif

#endif	/* RM_COMM_H */

/*-- end of file --*/
