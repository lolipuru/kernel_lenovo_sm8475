/*****************************************************************************
* File: RogueEnvConfig.h
*                                                                             
* (c) 2019 Sentons Inc. - All Rights Reserved.                           
*                                                                             
* All information contained herein is and remains the property of Sentons     
* Incorporated and its suppliers if any. The intellectual and technical       
* concepts contained herein are proprietary to Sentons Incorporated and its   
* suppliers and may be covered by U.S. and Foreign Patents, patents in        
* process, and are protected by trade secret or copyright law. Dissemination  
* of this information or reproduction of this material is strictly forbidden  
* unless prior written permission is obtained from Sentons Incorporated.      
*                                                                             
*****************************************************************************/

#ifndef _ROGUE_ENV_CONFIG_H_INCLUDED
#define _ROGUE_ENV_CONFIG_H_INCLUDED

// A collection of constants shared by the defining header files:
// SentonsTFifoReportConfig.h
// RogueFilterChainEnvAPI.h

// Number of squeeze detectors, squeeze configuration bank blocks,
// and squeeze entries in the squeeze gesture report
#ifndef SNT_GS_SQUEEZE_MAX_NUM
#define SNT_GS_SQUEEZE_MAX_NUM    2
#endif

// Maximum number of slider gesture reports
#ifndef GS_SLIDER_REC_MAX
#define GS_SLIDER_REC_MAX     3
#endif

// Number of tracks supported
#ifndef NUM_BAR_TOUCHES_SUPPORTED
#define NUM_BAR_TOUCHES_SUPPORTED (12)
#endif

#endif /* #ifndef _ROGUE_ENV_CONFIG_H_INCLUDED */



