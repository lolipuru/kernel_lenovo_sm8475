/****************************************************************************
* File: SentonsTFifoReport.h
*                                                                             
* (c) 2018 Sentons Inc. - All Rights Reserved.                           
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

#ifndef _SENTONS_TFIFO_REPORT_CONFIG_H_INCLUDED
#define _SENTONS_TFIFO_REPORT_CONFIG_H_INCLUDED

/*****************************************************************************
* 
*/
#include "RogueEnvConfig.h"    // shared constants


typedef struct bar1d_host_tr_rec_s 
{
  uint8_t   bar_trk_id;   // bar_id and trk_id
  uint8_t   force_lvl;   	// force of touch
  int16_t   pos0;         // center
  int16_t   pos1;         // top
  int16_t   pos2;         // bottom

} bar1d_host_tr_rec_t, *p_bar1d_host_tr_rec_t;

#define BAR1D_HOST_TR_REC_SIZE  (sizeof(struct bar1d_host_tr_rec_s))

// number of sprite force_lvl values supported
#define BAR1D_HOST_TR_SPRITE_REC_MAX  ((int)(BAR1D_HOST_TR_REC_SIZE - sizeof(uint8_t)))

typedef struct bar1d_host_tr_sprite_rec_s
{
  uint8_t   bar_trk_id;   // bar_id and trk_id
  uint8_t   force_lvl[BAR1D_HOST_TR_SPRITE_REC_MAX];
} bar1d_host_tr_sprite_rec_t, *p_bar1d_host_tr_sprite_rec_t;

#define BAR1D_HOST_TR_SPRITE_REC_SIZE (sizeof(struct bar1d_host_tr_sprite_rec_s))


#define BAR1D_HOST_TR_BAR_ID_POS    (5)
#define BAR1D_HOST_TR_TRK_ID_POS    (0)
#define BAR1D_HOST_TR_BAR_ID_WIDTH  (3)
#define BAR1D_HOST_TR_TRK_ID_WIDTH  (5)
#define BAR1D_HOST_TR_TRK_ID_MASK   ((1<<BAR1D_HOST_TR_TRK_ID_WIDTH)-1)
#define BAR1D_HOST_TR_BAR_ID_MASK   ((1<<BAR1D_HOST_TR_BAR_ID_WIDTH)-1)
#define BAR1D_HOST_TR_BAR_ID_FMASK  (BAR1D_HOST_TR_BAR_ID_MASK<<BAR1D_HOST_TR_BAR_ID_POS)

#define BAR1D_HOST_TR_BAR_ID_EXTRACT(a) (((a)>>BAR1D_HOST_TR_BAR_ID_POS)&BAR1D_HOST_TR_BAR_ID_MASK)
#define BAR1D_HOST_TR_TRK_ID_EXTRACT(a) (((a)>>BAR1D_HOST_TR_TRK_ID_POS)&BAR1D_HOST_TR_TRK_ID_MASK)


typedef struct bar1d_host_tr_s
{
  uint16_t                      length;   // in bytes, not including length parm
  uint16_t                      fr_nr;
  struct bar1d_host_tr_rec_s    tr[NUM_BAR_TOUCHES_SUPPORTED];

} bar1d_host_tr_t, *p_bar1d_host_tr_t;



/*****************************************************************************
* Gesture Reports may be added to struct bar1d_host_tr_s using the following
* format:
* 
* 1. gesture records will conform to the 8 byte size.
* 2. First gesture record will have first two bytes of 0x00 0x80
* 3. All subsequent gesture records will have first byte >0x80
*
* Gesture Header Record:
*
*    7  6  5  4  3  2  1  0
*  +-----------------------+
*  |          0x00         |
*  +-----------------------+
*  |          0x80         |
*  +-----------------------+
*  |    swipe0_velocity    |
*  +-----------------------+
*  |    swipe1_velocity    |
*  +-----------------------+
*  |    swipe2_velocity    |
*  +-----------------------+
*  |         tap           |
*  +-----------------------+
*  |      dbtap_misc       |
*  +--+--+--+--+-----------+
*  |Lo|Sh|Ca|St|En|        |
*  +--+--+--+--+--+--------+
*
* DEPRACATED:
* Lo - long squeeze indicator
* Sh - short squeeze indicator
* Ca - squeeze cancelled indicator
* St - squeeze start indicator
* En - squeeze end indicator
*/

#define GS_ID_HDR           0x80
#define GS_ID_SWIPE         0x80
#define GS_ID_TAP           0x80
#define GS_ID_DBTAP         0x80
#define GS_ID_SLIDE         0x81
#define GS_ID_SQUEEZE       0x82
#define GS_ID_MAX           0xff

#define GS_MASK(size, type)  ((type) ((1 << (size)) - 1))
#define GS_PMASK(mask, pos)  ((mask) << (pos))
#define GS_GET_FIELD(val, pos, mask) (((val) >> (pos)) & (mask))

#define GS_HDR_TAP0_START_POS         0
#define GS_HDR_TAP0_START_SIZE        1
#define GS_HDR_TAP0_START_MASK        GS_MASK(GS_HDR_TAP0_START_SIZE, uint8_t)
#define GS_HDR_TAP0_START_PMASK       GS_PMASK(GS_HDR_TAP0_START_MASK, GS_HDR_TAP0_START_POS)
#define GS_GET_TAP0_START(s)          GS_GET_FIELD(s, GS_HDR_TAP0_START_POS, GS_HDR_TAP0_START_MASK)

// Indexed versions of TAP_START
// p - tap_id
#define GS_GET_TAP_START_PMASK(p)    ((uint16_t)(GS_HDR_TAP0_START_MASK << (GS_HDR_TAP0_START_POS+(p))))
#define GS_GET_TAP_START(s,p)        (((s) & GS_GET_TAP_START_PMASK(p)) >> (GS_HDR_TAP0_START_POS+(p)))
#define GS_SET_TAP_START(s,p,v)      (((s) & ~GS_GET_TAP_START_PMASK(p)) | (((v) & GS_HDR_TAP0_START_MASK) << (GS_HDR_TAP0_START_POS+(p))))


#define GS_HDR_TAP0_STOP_POS          4
#define GS_HDR_TAP0_STOP_SIZE         1
#define GS_HDR_TAP0_STOP_MASK         GS_MASK(GS_HDR_TAP0_STOP_SIZE, uint8_t)
#define GS_HDR_TAP0_STOP_PMASK        GS_PMASK(GS_HDR_TAP0_STOP_MASK, GS_HDR_TAP0_STOP_POS)
#define GS_GET_TAP0_STOP(s)           GS_GET_FIELD(s, GS_HDR_TAP0_STOP_POS, GS_HDR_TAP0_STOP_MASK)

// Indexed versions of TAP_STOP
// p - tap_id
#define GS_GET_TAP_STOP_PMASK(p)     ((uint16_t)(GS_HDR_TAP0_STOP_MASK << (GS_HDR_TAP0_STOP_POS+(p))))
#define GS_GET_TAP_STOP(s,p)         (((s) & GS_GET_TAP_STOP_PMASK(p)) >> (GS_HDR_TAP0_STOP_POS+(p)))
#define GS_SET_TAP_STOP(s,p,v)       (((s) & ~GS_GET_TAP_STOP_PMASK(p)) | (((v) & GS_HDR_TAP0_STOP_MASK) << (GS_HDR_TAP0_STOP_POS+(p))))

// Double tap bits
#define GS_HDR_DBTAP0_POS         0
#define GS_HDR_DBTAP0_SIZE        1
#define GS_HDR_DBTAP0_MASK        GS_MASK(GS_HDR_DBTAP0_SIZE, uint8_t)
#define GS_HDR_DBTAP0_PMASK       GS_PMASK(GS_HDR_DBTAP0_MASK, GS_HDR_DBTAP0_POS)
#define GS_GET_DBTAP0(s)          GS_GET_FIELD(s, GS_HDR_DBTAP0_POS, GS_HDR_DBTAP0_MASK)
// Indexed versions of Double Tap
// p - double tap index
#define GS_GET_DBTAP_PMASK(p)     ((uint16_t)(GS_HDR_DBTAP0_MASK << (GS_HDR_DBTAP0_POS+(p))))
// Get Double_Tap[p] from byte s
#define GS_GET_DBTAP(s,p)         (((s) & GS_GET_DBTAP_PMASK(p)) >> (GS_HDR_DBTAP0_POS+(p)))
// Set Double_Tap[p] = v in byte s
#define GS_SET_DBTAP(s,p,v)       (((s) & ~GS_GET_DBTAP_PMASK(p)) | (((v) & GS_HDR_DBTAP0_MASK) << (GS_HDR_DBTAP0_POS+(p))))

#define GS_HDR_DBTAP0_STP_POS         4
#define GS_HDR_DBTAP0_STP_SIZE        1
#define GS_HDR_DBTAP0_STP_MASK        GS_MASK(GS_HDR_DBTAP0_STP_SIZE, uint8_t)
#define GS_HDR_DBTAP0_STP_PMASK       GS_PMASK(GS_HDR_DBTAP0_STP_MASK, GS_HDR_DBTAP0_STP_POS)
#define GS_GET_DBTAP0_STP(s)          GS_GET_FIELD(s, GS_HDR_DBTAP0_STP_POS, GS_HDR_DBTAP0_STP_MASK)
// Indexed versions of Double Tap
// p - double tap stop index
#define GS_GET_DBTAP_STP_PMASK(p)     ((uint16_t)(GS_HDR_DBTAP0_STP_MASK << (GS_HDR_DBTAP0_STP_POS+(p))))
// Get Double_Tap[p] stop from byte s
#define GS_GET_DBTAP_STP(s,p)         (((s) & GS_GET_DBTAP_STP_PMASK(p)) >> (GS_HDR_DBTAP0_STP_POS+(p)))
// Set Double_Tap[p] stp = v in byte s
#define GS_SET_DBTAP_STP(s,p,v)       (((s) & ~GS_GET_DBTAP_STP_PMASK(p)) | (((v) & GS_HDR_DBTAP0_STP_MASK) << (GS_HDR_DBTAP0_STP_POS+(p))))

// TODO: Clean up
// TODO: not needed?
// #define GS_SQUEEZE_REC_MAX    1

typedef struct __attribute__((__packed__)) gs_hdr_rec_s 
{
	uint8_t		escape0;
	uint8_t		escape1;
	int8_t		swipe0_velocity;
	int8_t		swipe1_velocity;
	int8_t		swipe2_velocity;
	uint8_t   tap;
	uint8_t   dbtap_misc;    // holds double_tap + reserve bits bits: 7:3=reserved, 2=dbtap[2], 1=dbtap[1], 0=dbtap[0]
	uint8_t   squeeze;  // NOTE: DEPRACATED replaced with new squeeze gesture report.
			    //       only set by code built with SUPPORT_LEGACY_GS_SQZ_V1 flag
} gs_hdr_rec_t, *p_gs_hdr_rec_t;

/*****************************************************************************
*
* Gesture Generic Records:
*
* All Gesture report records will have this basic format
*
*/


#define GS_BASE_REC_DATA_MAX        7

typedef struct __attribute__((__packed__)) gs_base_rec_s
{
  uint8_t   escape0;
  uint8_t   data[GS_BASE_REC_DATA_MAX];

} gs_base_rec_t, *p_gs_base_rec_t;

/*****************************************************************************
*
* Gesture Slider Records:
*
*    7  6  5  4  3  2  1  0
*  +-----------------------+
*  |          0x81         |
*  +-----------------------+
*  |fid1 | id1 |fid0 | id0 |
*  +-----------------------+
*  |     slider_force0     |
*  +-----------------------+
*  |     slider_force1     |
*  +-----------------------+
*  |      slider_pos0      |
*  |                       |
*  +-----------------------+
*  |      slider_pos1      |
*  |                       |
*  +-----------------------+
*
*/


#define GS_SLIDER_ID0_POS           0
#define GS_SLIDER_ID0_SIZE          2
#define GS_SLIDER_ID0_MASK          GS_MASK(GS_SLIDER_ID0_SIZE, uint8_t)
#define GS_SLIDER_ID0_PMASK         GS_PMASK(GS_SLIDER_ID0_MASK, GS_SLIDER_ID0_POS)
#define GS_GET_SLIDER_ID0(s)        (GS_GET_FIELD(s, GS_SLIDER_ID0_POS, GS_SLIDER_ID0_MASK))

#define GS_SLIDER_FID0_POS          2
#define GS_SLIDER_FID0_SIZE         2
#define GS_SLIDER_FID0_MASK         GS_MASK(GS_SLIDER_FID0_SIZE, uint8_t)
#define GS_SLIDER_FID0_PMASK        GS_PMASK(GS_SLIDER_FID0_MASK, GS_SLIDER_FID0_POS)
#define GS_GET_SLIDER_FID0(s)       (GS_GET_FIELD(s, GS_SLIDER_FID0_POS, GS_SLIDER_FID0_MASK))

#define GS_SLIDER_ID1_POS           4
#define GS_SLIDER_ID1_SIZE          2
#define GS_SLIDER_ID1_MASK          GS_MASK(GS_SLIDER_ID1_SIZE, uint8_t)
#define GS_SLIDER_ID1_PMASK         GS_PMASK(GS_SLIDER_ID1_MASK, GS_SLIDER_ID1_POS)
#define GS_GET_SLIDER_ID1(s)        (GS_GET_FIELD(s, GS_SLIDER_ID1_POS, GS_SLIDER_ID1_MASK))

#define GS_SLIDER_FID1_POS          6
#define GS_SLIDER_FID1_SIZE         2
#define GS_SLIDER_FID1_MASK         GS_MASK(GS_SLIDER_FID1_SIZE, uint8_t)
#define GS_SLIDER_FID1_PMASK        GS_PMASK(GS_SLIDER_FID1_MASK, GS_SLIDER_FID1_POS)
#define GS_GET_SLIDER_FID1(s)       (GS_GET_FIELD(s, GS_SLIDER_FID1_POS, GS_SLIDER_FID1_MASK))


typedef struct __attribute__((__packed__)) gs_slider_rec_s 
{
	uint8_t		escape0;
	uint8_t		slider_finger_id;
	uint8_t		slider_force0;
	uint8_t		slider_force1;
	int16_t		slider_pos0; // Signed to support relative positions
	int16_t		slider_pos1; // Signed to support relative positions

} gs_slider_rec_t, *p_gs_slider_rec_t;


/*****************************************************************************
*
* Gesture Squeeze Records:
* (assuming SNT_GS_SQUEEZE_MAX_NUM is 4)
*
*    7  6  5  4  3  2  1  0
*  +-----------------------+
*  |          0x82         |
*  +--+--+--+--+--+--------+
*  |Lo|Sh|Ca|St|En|        |    squeeze[0]
*  +--+--+--+--+--+--------+
*  |Lo|Sh|Ca|St|En|        |    squeeze[1]
*  +--+--+--+--+--+--------+
*  |Lo|Sh|Ca|St|En|        |    squeeze[2]
*  +--+--+--+--+--+--------+
*  |Lo|Sh|Ca|St|En|        |    squeeze[3]
*  +--+--+--+--+--+--------+
*  | Reserved              |
*  +-----------------------+
*  | Reserved              |
*  +-----------------------+
*  | Reserved              |
*  +-----------------------+
*
* Lo - long squeeze indicator
* Sh - short squeeze indicator
* Ca - squeeze cancelled indicator
* St - squeeze start indicator
* En - squeeze end indicator
*/

#define GS_HDR_SQUEEZE_END_POS        3
#define GS_HDR_SQUEEZE_END_SIZE       1
#define GS_HDR_SQUEEZE_END_MASK       GS_MASK(GS_HDR_SQUEEZE_END_SIZE, uint8_t)
#define GS_HDR_SQUEEZE_END_PMASK      GS_PMASK(GS_HDR_SQUEEZE_END_MASK, GS_HDR_SQUEEZE_END_POS)
#define GS_GET_SQUEEZE_END(s)         (GS_GET_FIELD(s, GS_HDR_SQUEEZE_END_POS, GS_HDR_SQUEEZE_END_MASK))

#define GS_HDR_SQUEEZE_START_POS      4
#define GS_HDR_SQUEEZE_START_SIZE     1
#define GS_HDR_SQUEEZE_START_MASK     GS_MASK(GS_HDR_SQUEEZE_START_SIZE, uint8_t)
#define GS_HDR_SQUEEZE_START_PMASK    GS_PMASK(GS_HDR_SQUEEZE_START_MASK, GS_HDR_SQUEEZE_START_POS)
#define GS_GET_SQUEEZE_START(s)       (GS_GET_FIELD(s, GS_HDR_SQUEEZE_START_POS, GS_HDR_SQUEEZE_START_MASK))

#define GS_HDR_SQUEEZE_CANCEL_POS      5
#define GS_HDR_SQUEEZE_CANCEL_SIZE     1
#define GS_HDR_SQUEEZE_CANCEL_MASK     GS_MASK(GS_HDR_SQUEEZE_CANCEL_SIZE, uint8_t)
#define GS_HDR_SQUEEZE_CANCEL_PMASK    GS_PMASK(GS_HDR_SQUEEZE_CANCEL_MASK, GS_HDR_SQUEEZE_CANCEL_POS)
#define GS_GET_SQUEEZE_CANCEL(s)       (GS_GET_FIELD(s, GS_HDR_SQUEEZE_CANCEL_POS, GS_HDR_SQUEEZE_CANCEL_MASK))

#define GS_HDR_SQUEEZE_SHORT_POS      6
#define GS_HDR_SQUEEZE_SHORT_SIZE     1
#define GS_HDR_SQUEEZE_SHORT_MASK     GS_MASK(GS_HDR_SQUEEZE_SHORT_SIZE, uint8_t)
#define GS_HDR_SQUEEZE_SHORT_PMASK    GS_PMASK(GS_HDR_SQUEEZE_SHORT_MASK, GS_HDR_SQUEEZE_SHORT_POS)
#define GS_GET_SQUEEZE_SHORT(s)       (GS_GET_FIELD(s, GS_HDR_SQUEEZE_SHORT_POS, GS_HDR_SQUEEZE_SHORT_MASK))

#define GS_HDR_SQUEEZE_LONG_POS      7
#define GS_HDR_SQUEEZE_LONG_SIZE     1
#define GS_HDR_SQUEEZE_LONG_MASK     GS_MASK(GS_HDR_SQUEEZE_LONG_SIZE, uint8_t)
#define GS_HDR_SQUEEZE_LONG_PMASK    GS_PMASK(GS_HDR_SQUEEZE_LONG_MASK, GS_HDR_SQUEEZE_LONG_POS)
#define GS_GET_SQUEEZE_LONG(s)       (GS_GET_FIELD(s, GS_HDR_SQUEEZE_LONG_POS, GS_HDR_SQUEEZE_LONG_MASK))

typedef struct __attribute__((__packed__)) gs_squeeze_rec_s
{
       uint8_t         escape0;
  uint8_t   squeeze[SNT_GS_SQUEEZE_MAX_NUM];
#if SNT_GS_SQUEEZE_MAX_NUM < 7
       uint8_t         reserved[7-SNT_GS_SQUEEZE_MAX_NUM];  // filler to keep the structure size at 8 bytes
#endif
} gs_squeeze_rec_t, *p_gs_squeeze_rec_t;

/*****************************************************************************/

/*****************************************************************************
* GS_RPT_MAX - 7 is the maximum number of gs+track reports and still allow for
* 64 bytes of tr_diag in a 128 byte upper bound for TFIFO Reports. 
*****************************************************************************/
#define GS_RPT_MAX     (7)

typedef struct __attribute__((__packed__)) gs_rpt_s 
{
  uint16_t        length;
  uint16_t        fr_nr;    // not used
	gs_base_rec_t	rec[GS_RPT_MAX];

} gs_rpt_t, *p_gs_rpt_t;


#define TR_DIAG_TYPE_HEALTHCHECK             0
#define TR_DIAG_TYPE_NTM                     1
#define TR_DIAG_TYPE_GPIO                    2
#define TR_DIAG_TYPE_MAX

/*****************************************************************************
 * Diag parameters may be added to struct bar1d_host_tr_s using the following
 * format:
 *
 * 1. All diag records are appended after the last real bar1d_host_tr_rec_s.
 * 2. The transition between bar1d_host_tr_rec_s and diag part is flagged by 
 *    the first byte of the diag area is set to 0x00. The diag area starts on
 *    a bar1d_host_tr_rec_s boundary. This is as if the bar_trk_id field is 0.
 * 3. The second byte of the diag section is the Version Id, currently 0x01.
 *    a. Version Id < 0x80 so as not to confuse with Gesture Reports
 * 4. The remaining bytes are a fixed structure whose format is defined by
 *    the version number.
 * 5. The diag structure plus the 0x00 and version field will always be a 
 *    multiple of 8. This, along with the 0x00 designator will allow the 
 *    diagnostic structure to coexist with the track_reports and be appended 
 *    to the end of the track reports.
 * 5. The maximum size of the diag report is 48 bytes (including 0x00 flag and
 *    version field). This corresponds to:
 *
 *    struct bar1d_host_tr_s
 *    {
 *      uint16_t                      length;     // includes length of diag
 *      uint16_t                      fr_nr;
 *      struct bar1d_host_tr_rec_s    tr[12];     // Maximum
 *      struct bar1d_tr_diag_xxxx_s   diag;
 *    };
 *
 *    struct bar1d_tr_diag_xxxx_s {
 *      uint8_t       start_code;     // always 0x00
 *      uint8_t       version;        // defines format of this structure
 *      ...                           // diagnostic fields
 *      uint8_t       padding[];      // optional to make multiple of 8 bytes
 *    }
 *
 *    NOTE: diag structure will be appended at end of however many track 
 *          reports are present.
 *
 *
 ****************************************************************************/

// TODO: move to RogueEnvConfig.h ?
#define NUM_BAR_DIAG_TOUCHES_SUPPORTED   (NUM_BAR_TOUCHES_SUPPORTED+6)
#define BAR_DIAG_HOST_TR_SIZE            (2*sizeof(uint16_t) + NUM_BAR_DIAG_TOUCHES_SUPPORTED*sizeof(struct bar1d_host_tr_rec_s))

#define TR_DIAG_START_CODE                0x00

#define TR_DIAG_REC_VERS_NONE             0x00
#define TR_DIAG_REC_VERS_001              0x01
#define TR_DIAG_REC_VERS_002              0x02
#define TR_DIAG_REC_VERS_TAP              0x03
#define TR_DIAG_REC_VERS_SLIDE            0x04
#define TR_DIAG_REC_VERS_TAP_SLIDE        0x05

#define TR_DIAG_REC_VERS_MAX              0x7f

#ifndef __min
#define __min(a,b) (((a)<(b)) ? (a) : (b))
#endif

#define TR_DIAG_MAX_MPATH                 10    // num in PH_0006_LP_Pixie

#ifdef SBOOT_SRC
#define TR_DIAG_NUM_PATH                  TR_DIAG_MAX_PATH
#else
#define TR_DIAG_NUM_MPATH                 __min(conf::glob_t::total_number_mpaths, TR_DIAG_MAX_MPATH)
#endif

typedef struct tr_diag_vers_001_s
{
    uint8_t           start_code;       // always 0x00
    uint8_t           vers;             // 0x01 in this case
    uint16_t          mpa[TR_DIAG_MAX_MPATH];     // D1
    uint8_t           d_mp[TR_DIAG_MAX_MPATH];    // D1
    uint8_t           ntm;                        // D1
    uint8_t           gpio;                       // C2
    uint8_t           atc;                        // C2
    uint8_t           rsvd[5];

} tr_diag_vers_001_t, *p_tr_diag_vers_001_t;

#ifndef SBOOT_SRC
STATIC_ASSERT(sizeof(tr_diag_vers_001_t)%8==0,"tr_diag Size must be multiple of 8");
#endif 

#define TR_DIAG_VERS_001_NUM_REC      (sizeof(tr_diag_vers_001_t)/sizeof(struct bar1d_host_tr_rec_s))

typedef struct tr_diag_vers_002_s
{
    uint8_t           start_code;       // always 0x00
    uint8_t           vers;             // 0x02 in this case
    uint16_t          mpa[TR_DIAG_MAX_MPATH];     // D1
    uint8_t           d_mp[TR_DIAG_MAX_MPATH];    // D1
    uint8_t           ntm;                        // D1
    uint8_t           gpio;                       // C2
    uint8_t           atc;                        // C2
    uint8_t           frame_rate;                 // C2
    uint8_t           trig;                       // C2
    uint8_t           rsvd[3];

} tr_diag_vers_002_t, *p_tr_diag_vers_002_t;

#define TR_DIAG_VERS_002_NUM_REC      (sizeof(tr_diag_vers_002_t)/sizeof(struct bar1d_host_tr_rec_s))

typedef struct tr_diag_tap_instance_s
{
    uint8_t           state;
    uint8_t           last_force;
    uint8_t           peak_force;
    uint8_t           floor_force;
    uint8_t           det_track_id;
    uint8_t           reserved1;
    uint16_t          window;

} tr_diag_tap_instance_t, *p_tr_diag_tap_instance_t;

// TODO: move to RogueEnvConfig.h ?
#define TR_DIAG_MAX_TAP     4

typedef struct tr_diag_tap_s
{
    uint8_t                 start_code;       // always 0x00
    uint8_t                 vers;             // 0x03 in this case
    uint8_t                 gpio;                       // C2
    uint8_t                 frame_rate;                 // C2
    uint8_t                 trig;                       // C2
    uint8_t                 filler1;               // 2 byte alignment
    tr_diag_tap_instance_t  tap[TR_DIAG_MAX_TAP];       // C2 Tap[i] gesture
    uint8_t                 filler2[2];                 // 

} tr_diag_tap_t, *p_tr_diag_tap_t;

#define TR_DIAG_TAP_NUM_REC         (sizeof(tr_diag_tap_t)/sizeof(struct bar1d_host_tr_rec_s))


typedef struct tr_diag_slide_instance_s
{
    uint16_t          pos0;
    uint8_t           state;
    uint8_t           bar_track;
    uint8_t           force0;
    uint8_t           max_dforce;
    uint8_t           nreport;
    uint8_t           reserved;

} tr_diag_slide_instance_t, *p_tr_diag_slide_instance_t;
    
#define TR_DIAG_MAX_SLIDE     3

typedef struct tr_diag_slide_s
{
    uint8_t                   start_code;               // always 0x00
    uint8_t                   vers;                     // 0x04 in this case
    uint8_t                   gpio;                     // C2
    uint8_t                   frame_rate;               // C2
    uint8_t                   trig;                     // C2
    uint8_t                   filler[3];                //  8-byte alignment
    tr_diag_slide_instance_t  slide[TR_DIAG_MAX_SLIDE];   // C2 Slide[i] gesture

} tr_diag_slide_t, *p_tr_diag_slide_t;

#define TR_DIAG_SLIDE_NUM_REC         (sizeof(tr_diag_slide_t)/sizeof(struct bar1d_host_tr_rec_s))


typedef struct tr_diag_tap_slide_s
{
    uint8_t                   start_code;               // always 0x00
    uint8_t                   vers;                     // 0x05 in this case
    uint8_t                   gpio;                     // C2
    uint8_t                   frame_rate;               // C2
    uint8_t                   trig;                     // C2
    uint8_t                   filler[3];                //  8-byte alignment
    tr_diag_tap_instance_t    tap[TR_DIAG_MAX_TAP];     // C2 Tap[i] gesture
    tr_diag_slide_instance_t  slide[TR_DIAG_MAX_SLIDE];   // C2 Slide[i] gesture

} tr_diag_tap_slide_t, *p_tr_diag_tap_slide_t;

#ifndef TR_DIAG_REC_VERS
#define TR_DIAG_REC_VERS    TR_DIAG_REC_VERS_002
#endif

#if TR_DIAG_REC_VERS == TR_DIAG_REC_VERS_002

#define tr_diag_rpt_t       tr_diag_vers_002_t
#define p_tr_diag_rpt_t     p_tr_diag_vers_002_t
#define TR_DIAG_NUM_REC     TR_DIAG_VERS_002_NUM_REC

#elif TR_DIAG_REC_VERS == TR_DIAG_REC_VERS_TAP

#define tr_diag_rpt_t       tr_diag_tap_t
#define p_tr_diag_rpt_t     p_tr_diag_tap_t
#define TR_DIAG_NUM_REC     TR_DIAG_TAP_NUM_REC

#else

#error "TR_DIAG_REC_VERS UNDEFINED"

#endif // TR_DIAG_REC_VERS




#endif /* #ifndef _SENTONS_TFIFO_REPORT_CONFIG_H_INCLUDED */
