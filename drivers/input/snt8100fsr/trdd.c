/*****************************************************************************
* File: trdd.c
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
* SENTONS PROVIDES THIS SOURCE CODE STRICTLY ON AN "AS IS" BASIS,
* WITHOUT ANY WARRANTY WHATSOEVER, AND EXPRESSLY DISCLAIMS ALL
* WARRANTIES, EXPRESS, IMPLIED OR STATUTORY WITH REGARD THERETO, INCLUDING
* THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
* PURPOSE, TITLE OR NON-INFRINGEMENT OF THIRD PARTY RIGHTS. SENTONS SHALL
* NOT BE LIABLE FOR ANY DAMAGES SUFFERED BY YOU AS A RESULT OF USING,
* MODIFYING OR DISTRIBUTING THIS SOFTWARE OR ITS DERIVATIVES.
*
*
*****************************************************************************/

//
// BUILD LINE FOR THIS TOOL:
//
// gcc -o trdd trdd.c
//

/*****************************************************************************
 * INCLUDE FILES
 ****************************************************************************/
#include <unistd.h>
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <getopt.h>
#include <string.h>
#include "track_report.h"
//#define SBOOT_SRC
//#include <SentonsTFifoReportConfig.h>

/*****************************************************************************
 * MACROS AND DATA STRUCTURES
 ****************************************************************************/

typedef struct input_tr_rec_s
{
    uint32_t ts;
    uint32_t fn;
    uint32_t bar_id;
    uint32_t a1;
    uint32_t a2;
    uint32_t a3;
    uint32_t a4;
    uint32_t a5;
    uint32_t a6;
    uint32_t a7;
    uint32_t a8;
    uint32_t a9;
    uint32_t a10;
    uint32_t a11;
    uint32_t a12;
    uint32_t a13;
    uint32_t a14;
    uint32_t a15;
    uint32_t a16;
    uint32_t a17;
    uint32_t a18;
    uint32_t a19;
    uint32_t a20;
    uint32_t a21;
    uint32_t a22;

} input_tr_rec_t, *p_input_tr_rec_t;


typedef struct event_log_s 
{
    uint8_t     sub_sys;
    uint8_t     evt_id;
    uint16_t    parm1;
    uint16_t    parm2;
    uint16_t    parm3;

} event_log_t, *p_event_log_t;

#define STATE_TR            0
#define STATE_TR_DIAG       1

#define MIN_STRAIN_BAR_ID   6
#define MAX_STRAIN_BAR_ID   7
#define IS_STRAIN_BAR(id)   ((id) <= MAX_STRAIN_BAR_ID && (id) >= MIN_STRAIN_BAR_ID)

#ifndef GS_TAP_MAX_NUM
#define GS_TAP_MAX_NUM      4
#endif

#ifndef GS_DTAP_MAX_NUM
#define GS_DTAP_MAX_NUM     3
#endif

#define FTYPE_NONE              0
#define FTYPE_TXT               1
#define FTYPE_BIN               2

#define EVENT_NONE              0
#define EVENT_CVT_TRBIN_2_DT    1
#define EVENT_LOG_BIN           2
#define EVENT_DEEP_TRACE        3
#define EVENT_TR_TXT_LEG        4
#define EVENT_TR_TXT            5
#define EVENT_TR_BIN            6


#define PRINT_LEVEL_NORMAL  0
#define PRINT_LEVEL_VERBOSE 1
#define PRINT_LEVEL_DEBUG   2

/*****************************************************************************
 * GLOBAL VARIABLES
 ****************************************************************************/

char *in_fname = NULL;
char *out_fname = NULL;
FILE *fin = NULL;
FILE *fout = NULL;
FILE *ferr = NULL;
int state = STATE_TR;
int tr_cur_rec;
int tr_max_rec;
int print_level = PRINT_LEVEL_NORMAL;

uint8_t *p_tr_diag_buf;
uint8_t *p_tr_diag_base;
tr_diag_vers_001_t tr_diag_rec_001;
tr_diag_vers_002_t tr_diag_rec_002;

int ftype = FTYPE_TXT;
int event = EVENT_TR_TXT;

int out_ftype = FTYPE_TXT;

uint32_t    tr_diag_build_txt[64];

uint32_t cur_ts = 0;

#define TAP_HISTO_NUM_BAR 8
int tap_histo_flag = 0;
int tap_histo_bar = -1;
int tap_histo_num[TAP_HISTO_NUM_BAR]={0,0,0,0,0,0,0,0};
int tap_histo_force[TAP_HISTO_NUM_BAR]={-1,-1,-1,-1,-1,-1,-1,-1};
int tap_histo_found[TAP_HISTO_NUM_BAR]={0,0,0,0,0,0,0,0};
int tap_histo_frame=0;
char tap_histo_tap_char[TAP_HISTO_NUM_BAR]={'*','*','*','*','*','*','*','*'};
char tap_histo_touch_char[TAP_HISTO_NUM_BAR] = {'.','.','.','.','.','.','.','.'};


/*****************************************************************************
 * FUNCTION DECLARATIONS
 ****************************************************************************/


void process_report_rec(p_input_tr_rec_t p_in);
void start_tr_diag(p_input_tr_rec_t p_in);
void process_tr_diag_legacy(p_input_tr_rec_t p_in);
void print_region_report_from_in_report(p_input_tr_rec_t p_in);
void print_position_report_from_in_report(p_input_tr_rec_t p_in);
void print_tr_diag_bin(uint8_t *p_buf);
void print_tr_diag_001(void* p_buf);
void print_tr_diag_002(void* p_buf);
void print_tr_diag_003(void* p_buf);
void print_tr_diag_003_histo(void* p_buf);
void print_tr_diag_004(void* p_buf);
void print_tr_diag_005(void* p_buf);
void process_tr_diag_txt_file(void);
void process_tr_diag_txt_file_legacy(void);
void process_tr_diag_bin_file(void);
void process_deep_trace_file(void);
void process_event_log_bin_file(void);
void convert_tr_bin_2_deep_trace(void);
void print_gs_hdr_txt(char *p);
void print_gs_slide_txt(char *p);
void print_gs_squeeze_txt(char *p);
void print_gs_hdr_bin(p_gs_hdr_rec_t p);
void process_tr_diag_txt(char *p);


/*****************************************************************************
 * 
 * SOURCE CODE - Main section
 * 
 ****************************************************************************/

/** 
 */
void print_usage() {
    printf("Usage: trdd [-cerdh] [-o outfile] [infile]\n");
    printf("       -c: convert track_report.bin file to deep_trace.bin file\n");
    printf("           -c -o deep_trace.bin track_report.bin\n");
    printf("       -d: print deep_trace.bin (binary) file\n");
    printf("       -e: print event_log.bin (binary) file\n");
    printf("       -h: print this help text\n");
    printf("       -r: print track_report.bin (binary) file\n");
    printf("       -v: verbose output\n");
    printf("       -l: print track_report.log (legacy text) file prior to 3.6.3\n");
    printf("       default: print track_report.log (text) file\n");
}


FILE *open_in_file(char *fname, int ftype)
{
    FILE *fret = NULL;
    if (fname == NULL) {
        fret = stdin;
    } else {
        switch (ftype) {
            case FTYPE_NONE:
            case FTYPE_TXT: fret = fopen(fname, "r");  break;
            case FTYPE_BIN: fret = fopen(fname, "rb"); break;
            default: fprintf(ferr,"Unknown ftype (%d)\n", ftype); break;
        }
    }
    return fret;
}

FILE *open_out_file(char *fname, int ftype)
{
    FILE *fret = NULL;

    if (fname == NULL) {
        fret = stdout;
    } else {
        switch (ftype) {
            case FTYPE_NONE :
            case FTYPE_TXT  : fret = fopen(fname, "w"); break;
            case FTYPE_BIN  : fret = fopen(fname, "wb"); break;
            default: fprintf(ferr,"Unknown ftype (%d)\n", ftype); break;
        }
    }
    return fret;
}

/*****************************************************************************
 * main()
 *
 * usage: trdd [track_report_file]
 *
 * If track_report_file not supplied, assumes stdin.
 *
 * input is track report logging file from snt8100fsr reference linux driver:
 *
 * echo 1 >/sys/snt8100fsr/log_track_reports
 * echo 0 >/sys/snt8100fsr/log_track_reports
 *
 * This program will parse a track_report log file and reformat the track
 * report diagnostic section.
 *
 * To turn on track report diagnostic logging:
 *
 * echo 0x40 1 >/sys/snt8100fsr/set_reg
 *
 * To Make:
 *
 * cc -o trdd trdd.c
 *
 ****************************************************************************/
int main(int argc, char *argv[])
{
    int option = 0;

    ferr = stderr;
  
     //Specifying the expected options
    while ((option = getopt(argc, argv,"cedhlo:rvH:")) != -1) {
        switch (option) {
            case 'c': event = EVENT_CVT_TRBIN_2_DT;
                      ftype = FTYPE_BIN; 
                      out_ftype = FTYPE_BIN; 
                      break;
            case 'e': event = EVENT_LOG_BIN;
                      ftype = FTYPE_BIN; 
                      break;
            case 'd': event = EVENT_DEEP_TRACE;
                      ftype = FTYPE_BIN; 
                      break;
            case 'l': event = EVENT_TR_TXT_LEG;
                      ftype = FTYPE_TXT; 
                      break;
            case 'o': out_fname = optarg; 
                      break;
            case 'r': event = EVENT_TR_BIN;
                      ftype = FTYPE_BIN; 
                      break;
            case 'v': print_level = PRINT_LEVEL_VERBOSE; break;
            case 'H': tap_histo_flag = 1; tap_histo_bar = atoi(optarg); break;
            default : print_usage(); exit(1); break;
        }
    }

    // Open input and output files
    if (optind < argc) {
        in_fname = argv[optind];
    } 
    fin = open_in_file(in_fname, ftype);
    if (fin==NULL) {
        fprintf(ferr, "ERROR! Couldn't open %s\n", in_fname);
        exit(1);
    }
    fout = open_out_file(out_fname, out_ftype);
    if (fout==NULL) {
        fprintf(ferr, "ERROR! Couldn't open %s\n", out_fname);
        exit(1);
    }
    switch (event) {
        case EVENT_LOG_BIN        : process_event_log_bin_file();       break;
        case EVENT_TR_TXT         : process_tr_diag_txt_file();         break;
        case EVENT_TR_TXT_LEG     : process_tr_diag_txt_file_legacy();  break;
        case EVENT_TR_BIN         : process_tr_diag_bin_file();         break;
        case EVENT_DEEP_TRACE     : process_deep_trace_file();          break;
        case EVENT_CVT_TRBIN_2_DT : if (!in_fname || !out_fname) {
                                      print_usage();
                                      exit(1);
                                    }
                                    convert_tr_bin_2_deep_trace();      
                                    break;
        default: fprintf(ferr,"Unknown ftype (%d)\n", ftype);           break;
    }
}

/*****************************************************************************
 * 
 * TRACK REPORT BINARY FORMAT PROCESSING
 * 
 ****************************************************************************/

#ifdef xTAP_HISTO
/*****************************************************************************
 * print_position_report_bin()
 * 
 * Print position track report from binary record
 *
 ****************************************************************************/
void print_position_report_bin(uint32_t ts, uint32_t fn, p_track_report_t p_tr_rec)
{
    if (p_tr_rec) {
        tap_histo_force[p_tr_rec->bar_id] = p_tr_rec->force_lvl;
    }
  return;
  (void) ts;
  (void) fn;
}
#else
/*****************************************************************************
 * print_position_report_bin()
 * 
 * Print position track report from binary record
 *
 ****************************************************************************/
void print_position_report_bin(uint32_t ts, uint32_t fn, p_track_report_t p_tr_rec)
{
    if (p_tr_rec) {
      if (tap_histo_flag) {
        if (tap_histo_bar == p_tr_rec->bar_id)
          tap_histo_force[p_tr_rec->bar_id] = p_tr_rec->force_lvl;
      } else {
        if (ts!=0 || fn!=0) fprintf(fout, "%u(%u): ", fn, ts);

        fprintf(fout,"bar=%u trk=%u frc=%u p0=%5.3f(%u) p1=%5.3f(%u) p2=%5.3f(%u)\n",
                 p_tr_rec->bar_id, 
                 p_tr_rec->trk_id, 
                 p_tr_rec->force_lvl, 
                 (float)p_tr_rec->center/16.0,p_tr_rec->center,
                 (float)p_tr_rec->bottom/16.0,p_tr_rec->bottom,
                 (float)p_tr_rec->top/16.0,p_tr_rec->top);
      }
    }
}
#endif

/*****************************************************************************
 * print_tr_diag_bin()
 * 
 * Print tr diag structure.
 * 
 ****************************************************************************/
void print_tr_diag_bin(uint8_t* p_buf)
{
    if (p_buf) {
      if (tap_histo_flag) {
        switch (p_buf[1]) {
            case 1: print_tr_diag_001((void*) p_buf); break;
            case 2: print_tr_diag_002((void*) p_buf); break;
            case 3: print_tr_diag_003_histo((void*) p_buf); break;
            case 4: print_tr_diag_004((void*) p_buf); break;
            case 5: print_tr_diag_005((void*) p_buf); break;
            default: fprintf(stderr, "ERROR! print! Unknown version %u\n", p_buf[1]);
                     exit(1);
        }
      } else {
        switch (p_buf[1]) {
            case 1: print_tr_diag_001((void*) p_buf); break;
            case 2: print_tr_diag_002((void*) p_buf); break;
            case 3: print_tr_diag_003((void*) p_buf); break;
            case 4: print_tr_diag_004((void*) p_buf); break;
            case 5: print_tr_diag_005((void*) p_buf); break;
            default: fprintf(stderr, "ERROR! print! Unknown version %u\n", p_buf[1]);
                     exit(1);
        }
      }
    }
}


/*****************************************************************************
 * print_tr_diag_001()
 *
 * formats and prints version 1 tr_diag.
 ****************************************************************************/
void print_tr_diag_001(void* p)
{
  int i;
  p_tr_diag_vers_001_t p_diag = (p_tr_diag_vers_001_t) p;

  fprintf(fout, "    vers=%d, gpio=0x%02x, atc=%d, ntm.stress=%d, ntm.nt=%d\n",
            p_diag->vers,
            p_diag->gpio,
            p_diag->atc,
            p_diag->ntm>>4,
            p_diag->ntm&0xf);
  fprintf(fout, "    mpa : ");
  for (i=0; i < TR_DIAG_MAX_MPATH; i++) {
    fprintf(fout, "0x%02x ",p_diag->mpa[i]);
  }
  fprintf(fout, "\n    d_mp: ");
  for (i=0; i < TR_DIAG_MAX_MPATH; i++) {
    fprintf(fout, "0x%04x ",p_diag->d_mp[i]);
  }
  fprintf(fout,"\n");
}


/*****************************************************************************
 * print_tr_diag_002()
 *
 * formats and prints version 2 tr_diag.
 ****************************************************************************/
void print_tr_diag_002(void* p)
{
  int i;
  p_tr_diag_vers_002_t p_diag = (p_tr_diag_vers_002_t) p;

  fprintf(fout, "    vers=%d, fr=%u, tap=0x%02x, gpio=0x%02x, atc=%d, ntm.stress=%d, ntm.nt=%d\n",
            p_diag->vers,
            p_diag->frame_rate,
            p_diag->trig,
            p_diag->gpio,
            p_diag->atc,
            p_diag->ntm>>4,
            p_diag->ntm&0xf);
  fprintf(fout, "    mpa : ");
  for (i=0; i < TR_DIAG_MAX_MPATH; i++) {
    fprintf(fout, "0x%04x ",p_diag->mpa[i]&0xffff);
  }
  fprintf(fout, "\n    d_mp: ");
  for (i=0; i < TR_DIAG_MAX_MPATH; i++) {
    fprintf(fout, "0x%02x ",p_diag->d_mp[i]&0xff);
  }
  fprintf(fout,"\n");
}

void print_tr_diag_tap_rec(void* p)
{
  p_tr_diag_tap_instance_t p_tap = (p_tr_diag_tap_instance_t) p;

  fprintf(fout, "        state=%d, trk_id=0x%02x, last=%d, peak=%d, floor=%d, window=0x%04x\n",
           p_tap->state,
           p_tap->det_track_id,
           p_tap->last_force,
           p_tap->peak_force,
           p_tap->floor_force,
           p_tap->window);
}

void print_tr_diag_slide_rec(void *p)
{
  p_tr_diag_slide_instance_t p_slide = (p_tr_diag_slide_instance_t) p;

  fprintf(fout, "        state=%d, bar_trk=0x%02x, p0=%d, f0=%d, maxdf=%d, nrep=%d\n",
          p_slide->state,
          p_slide->bar_track,
          p_slide->pos0,
          p_slide->force0,
          p_slide->max_dforce,
          p_slide->nreport);
}

void print_tr_diag_003_histo(void* p)
{
  int i,ibar;
  p_tr_diag_tap_t p_dg = (p_tr_diag_tap_t) p;

  // figure out if a bar has a new tap, continued tap, just touch, or no touch
  //       new tap tap_histo_found==0, trig&bar!=0
  //       cont tap tap_histo_found==1, trig&bar!=0
  //       just touch tap_histo_found=0, trig&bar==0, force[bar] >= 0
  //       no touch force[bar] < 0
  //
  //for (ibar=0; ibar<TAP_HISTO_NUM_BAR; ibar++) {
  if (tap_histo_flag) {
    ibar = tap_histo_bar;
    // check for new tap
    if (tap_histo_found[ibar]==0 && (p_dg->trig&(1<<ibar))) {
      tap_histo_found[ibar] = 1;
      tap_histo_num[ibar]++;
    }
    // check for no tap
    if ((p_dg->trig&(1<<ibar)) == 0) tap_histo_found[ibar] = 0;
    // check for touch
    if (tap_histo_force[ibar] >= 0) {
      fprintf(fout,"%5d %4d %1d:", tap_histo_frame, tap_histo_num[ibar], ibar);
      for (i=0; i < tap_histo_force[ibar]; i++) {
        fprintf(fout,"%c", (tap_histo_found[ibar]) ? tap_histo_tap_char[ibar] : tap_histo_touch_char[ibar]);
      }
      fprintf(fout," %d\n", tap_histo_force[ibar]);
    }
    // check for last touch use -1 to indicate no more touch
    if (tap_histo_force[ibar]==0) tap_histo_force[ibar]=-1;
  }
}




void print_tr_diag_003(void* p)
{
  int i;
  p_tr_diag_tap_t p_dg = (p_tr_diag_tap_t) p;

  fprintf(fout, "    vers=%d, fr=%u, tap=0x%02x, trig=0x%02x\n",
                    p_dg->vers,
                    p_dg->frame_rate,
                    p_dg->trig,
                    p_dg->gpio);
  for (i=0; i < TR_DIAG_MAX_TAP; i++) {
    print_tr_diag_tap_rec(&p_dg->tap[i]);
  }
}

void print_tr_diag_004(void* p)
{
  int i;
  p_tr_diag_slide_t p_dg = (p_tr_diag_slide_t) p;

  fprintf(fout, "    vers=%d, fr=%u, tap=0x%02x, trig=0x%02x\n",
                    p_dg->vers,
                    p_dg->frame_rate,
                    p_dg->trig,
                    p_dg->gpio);
  for (i=0; i < TR_DIAG_MAX_SLIDE; i++) {
    print_tr_diag_slide_rec(&p_dg->slide[i]);
  }
}

void print_tr_diag_005(void* p)
{
  int i;
  p_tr_diag_tap_slide_t p_dg = (p_tr_diag_tap_slide_t) p;
  fprintf(fout, "    vers=%d, fr=%u, tap=0x%02x, trig=0x%02x\n",
                    p_dg->vers,
                    p_dg->frame_rate,
                    p_dg->trig,
                    p_dg->gpio);
  for (i=0; i < TR_DIAG_MAX_TAP; i++) {
    print_tr_diag_tap_rec(&p_dg->tap[i]);
  }
  fprintf(fout, "\n");
  for (i=0; i < TR_DIAG_MAX_SLIDE; i++) {
    print_tr_diag_slide_rec(&p_dg->slide[i]);
  }
}

/*****************************************************************************
 * print_region_report_bin()
 * 
 * Print region track report from binary record
 *
 ****************************************************************************/
void print_region_report_bin(uint32_t ts, uint32_t fn, p_stg_track_report_t p_rg)
{
    if (p_rg) {
        if (ts!=0 || fn!=0) fprintf(fout, "%u(%u): ", fn, ts);

        fprintf(fout,
            "bar=%u f0=%u f1=%u f2=%u f3=%u f4=%u f5=%u f6=%u\n",
            p_rg->bar_id,
            p_rg->force_lvl[0], p_rg->force_lvl[1],
            p_rg->force_lvl[2], p_rg->force_lvl[3],
            p_rg->force_lvl[4], p_rg->force_lvl[5],
            p_rg->force_lvl[6]);
    }
}


/*****************************************************************************
 * print_track_rec_bin()
 * 
 * Print a track report from a binary record
 *
 ****************************************************************************/
void print_track_rec_bin(int fr_nr, p_track_report_t p_tr_rec, uint32_t ts)
{   
    if (p_tr_rec==NULL) return;

    if (IS_STRAIN_BAR(p_tr_rec->bar_id)) {
        print_region_report_bin(ts, fr_nr, (p_stg_track_report_t)p_tr_rec);
    } else {
        print_position_report_bin(ts, fr_nr, p_tr_rec);
    }
}


/*****************************************************************************
 * print_gs_hdr_bin()
 * 
 * Print the Gesture Header from a binary record
 *
 ****************************************************************************/
void print_gs_hdr_bin(p_gs_hdr_rec_t p)
{
    int i;
    if (p) {
        if (print_level >= PRINT_LEVEL_VERBOSE) {
            fprintf(fout, "    0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
                                ((uint8_t*)p)[0],
                                ((uint8_t*)p)[1],
                                ((uint8_t*)p)[2],
                                ((uint8_t*)p)[3],
                                ((uint8_t*)p)[4],
                                ((uint8_t*)p)[5],
                                ((uint8_t*)p)[6],
                                ((uint8_t*)p)[7]);
        }
        if (p->squeeze != 0) {
            // Handle the legacy case of the squeeze still being in the gesture header
            fprintf(fout, "    Gesture: sw0=%d sw1=%d sw2=%d, sq_st=%u, sq_cs=%u, sq_sh=%u, sq_lg=%u, sq_end=%u\n",
                        p->swipe0_velocity, p->swipe1_velocity, p->swipe2_velocity,
                        GS_GET_SQUEEZE_START(p->squeeze), 
                        GS_GET_SQUEEZE_CANCEL(p->squeeze), 
                        GS_GET_SQUEEZE_SHORT(p->squeeze), 
                        GS_GET_SQUEEZE_LONG(p->squeeze),
                        GS_GET_SQUEEZE_END(p->squeeze));

        } else {
            // Normal case of no squeeze in the the header
            fprintf(fout, "    Gesture: sw0=%d sw1=%d sw2=%d\n",
                        p->swipe0_velocity, p->swipe1_velocity, p->swipe2_velocity);
        }
        fprintf(fout, "             tap_start[");
        for (i=0;i<GS_TAP_MAX_NUM;i++)
            if (GS_GET_TAP_START(p->tap,i)) fprintf(fout,"%d",i);
        fprintf(fout, "] tap_stop[");
        for (i=0;i<GS_TAP_MAX_NUM;i++)
            if (GS_GET_TAP_STOP(p->tap,i)) fprintf(fout,"%d",i);
        fprintf(fout,"] (0x%x)\n", p->tap);
        
        fprintf(fout, "             dtap_start[");
        for (i=0;i<GS_DTAP_MAX_NUM;i++)
            if (GS_GET_TAP_START(p->dtap,i)) fprintf(fout,"%d",i);
        fprintf(fout, "] dtap_stop[");
        for (i=0;i<GS_DTAP_MAX_NUM;i++)
            if (GS_GET_TAP_STOP(p->dtap,i)) fprintf(fout,"%d",i);
        fprintf(fout,"] (0x%x)\n", p->dtap);
    }
}



/*****************************************************************************
 * print_gs_squeeze_bin()
 * 
 * Print the Gesture Squeeze report from a binary record
 *
 ****************************************************************************/
void print_gs_squeeze_bin(p_gs_squeeze_rec_t p_sq)
{ 
    if (p_sq) {
        fprintf(fout, "    Squeeze: (St|Ca|Sh|Lo|End)");
        for (int sqIdx = 0; sqIdx < SNT_GS_SQUEEZE_MAX_NUM; sqIdx++) {
            if (p_sq->squeeze[sqIdx] == 0) {
                fprintf(fout, " #%d=(-----)", sqIdx);
            } else {
                fprintf(fout, " #%d=(%d%d%d%d%d)",
                    sqIdx,
                    GS_GET_SQUEEZE_START(p_sq->squeeze[sqIdx]),
                    GS_GET_SQUEEZE_CANCEL(p_sq->squeeze[sqIdx]),
                    GS_GET_SQUEEZE_SHORT(p_sq->squeeze[sqIdx]),
                    GS_GET_SQUEEZE_LONG(p_sq->squeeze[sqIdx]), 
                    GS_GET_SQUEEZE_END(p_sq->squeeze[sqIdx]) );
            }
        }
        fprintf(fout,"\n");
    }
}




/*****************************************************************************
 * print_gs_slide_bin()
 * 
 * print slider gesture report from binary structure.
 * 
 */
void print_gs_slide_bin(p_gs_slider_rec_t p)
{
    if (p) {
        uint8_t id0 = GS_GET_SLIDER_ID0(p->slider_finger_id);
        uint8_t id1 = GS_GET_SLIDER_ID1(p->slider_finger_id);
        uint8_t fid0 = GS_GET_SLIDER_FID0(p->slider_finger_id);
        uint8_t fid1 = GS_GET_SLIDER_FID1(p->slider_finger_id);
        if (print_level >= PRINT_LEVEL_VERBOSE) {
            fprintf(fout, "    0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
                                ((uint8_t*)p)[0],
                                ((uint8_t*)p)[1],
                                ((uint8_t*)p)[2],
                                ((uint8_t*)p)[3],
                                ((uint8_t*)p)[4],
                                ((uint8_t*)p)[5],
                                ((uint8_t*)p)[6],
                                ((uint8_t*)p)[7]);
        }
        if (fid0) {
            fprintf(fout, "    Slider[%u,%u]: frc=%u, pos=%u\n", id0, fid0,
                            p->slider_force0, p->slider_pos0);
        }
        if (fid1) {
            fprintf(fout, "    Slider[%u,%u]: frc=%u, pos=%u\n", id1, fid1,
                            p->slider_force1, p->slider_pos1);
        }
    }
}

#ifdef xTAP_HISTO
/*****************************************************************************
 * print_tr_tap_histo_bin()
 * 
 * print TFIFO Report frame in binary format.
 * 
 */
void print_tr_bin(p_bar1d_host_tr_t p_tr, uint32_t ts)
{
    if (p_tr == NULL) {fprintf(ferr,"NULL tr record\n"); exit(1);}

    int num_rec = (p_tr->length - sizeof(uint16_t))/sizeof(track_report_t); // sub out fr_nr field

    int rec_idx = 0;
    int gs_rec_found = 0;
    int i;

    if (print_level >= PRINT_LEVEL_VERBOSE) {
      fprintf(fout, "TFIFO Report Number of Records = %d\n\n", num_rec);
      fprintf(fout, "TFIFO Report Raw Record dump:\n\n");
      for (i=0; i < num_rec; i++) {
        fprintf(fout,"    %2i: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", i,
                  ((uint8_t*)&p_tr->tr[i])[0],
                  ((uint8_t*)&p_tr->tr[i])[1],
                  ((uint8_t*)&p_tr->tr[i])[2],
                  ((uint8_t*)&p_tr->tr[i])[3],
                  ((uint8_t*)&p_tr->tr[i])[4],
                  ((uint8_t*)&p_tr->tr[i])[5],
                  ((uint8_t*)&p_tr->tr[i])[6],
                  ((uint8_t*)&p_tr->tr[i])[7]);
      }
      fprintf(fout,"\nTFIFO Report Formated Records:\n");
    }
    //fprintf(fout,"\n");

    while (rec_idx < num_rec) {
        tap_histo_frame = p_tr->fr_nr;

        // tr_diag section
        if (p_tr->tr[rec_idx].bar_id == 0 && 
            p_tr->tr[rec_idx].trk_id == 0 &&
            p_tr->tr[rec_idx].force_lvl <= TR_DIAG_REC_VERS_MAX) {
            print_tr_diag_bin((uint8_t*)&p_tr->tr[rec_idx]);

            break;  // end of records
        } 

        if (gs_rec_found || (p_tr->tr[rec_idx].bar_id == 0 &&
                             p_tr->tr[rec_idx].trk_id == 0 &&  
                             p_tr->tr[rec_idx].force_lvl >= GS_RPT_TYPE_HDR)) {
            // gesture section
            if (gs_rec_found == 0) {
                p_gs_hdr_rec_t p = (p_gs_hdr_rec_t) &p_tr->tr[rec_idx];
                if (rec_idx == 0) {
                }
                //print_gs_hdr_bin(p);
                gs_rec_found = 1;
            } else {
                // Gesture Report
                // First guess is slider.  If that's wrong then keep trying
               // p_gs_slider_rec_t p = (p_gs_slider_rec_t) &p_tr->tr[rec_idx];
               // if (p->escape0 == GS_RPT_TYPE_SLIDE) {
               //     print_gs_slide_bin(p);
               // } else if (p->escape0 == GS_RPT_TYPE_SQUEEZE) {
               //     print_gs_squeeze_bin((p_gs_squeeze_rec_t)p);
               // } else {      
               //     // This is not good.  Better to report it than to ignore it
               //     fprintf(fout,"    ERROR: Bad Gesture Report id (%d)\n", p->escape0);
               // }
            }

        } else {
            // track reports section
            print_track_rec_bin(p_tr->fr_nr, &p_tr->tr[rec_idx], ts);
        }
        rec_idx++;
    }
}
#else

/*****************************************************************************
 * print_tr_bin()
 * 
 * print TFIFO Report frame in binary format.
 * 
 */
void print_tr_bin(p_bar1d_host_tr_t p_tr, uint32_t ts)
{
    if (p_tr == NULL) {fprintf(ferr,"NULL tr record\n"); exit(1);}

    int num_rec = (p_tr->length - sizeof(uint16_t))/sizeof(track_report_t); // sub out fr_nr field

    int rec_idx = 0;
    int gs_rec_found = 0;
    int i;
    
    if (print_level >= PRINT_LEVEL_VERBOSE) {
      fprintf(fout, "TFIFO Report Number of Records = %d\n\n", num_rec);
      fprintf(fout, "TFIFO Report Raw Record dump:\n\n");
      for (i=0; i < num_rec; i++) {
        fprintf(fout,"    %2i: 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n", i,
                  ((uint8_t*)&p_tr->tr[i])[0],
                  ((uint8_t*)&p_tr->tr[i])[1],
                  ((uint8_t*)&p_tr->tr[i])[2],
                  ((uint8_t*)&p_tr->tr[i])[3],
                  ((uint8_t*)&p_tr->tr[i])[4],
                  ((uint8_t*)&p_tr->tr[i])[5],
                  ((uint8_t*)&p_tr->tr[i])[6],
                  ((uint8_t*)&p_tr->tr[i])[7]);
      }
      fprintf(fout,"\nTFIFO Report Formated Records:\n");
    }

    // if histogram cache frame number for print later, else start print now
    if (tap_histo_flag) 
      tap_histo_frame = p_tr->fr_nr;
    else
      fprintf(fout,"\n");

    while (rec_idx < num_rec) {

        // tr_diag section
        if (p_tr->tr[rec_idx].bar_id == 0 && 
            p_tr->tr[rec_idx].trk_id == 0 &&
            p_tr->tr[rec_idx].force_lvl <= TR_DIAG_REC_VERS_MAX) {
            if (rec_idx == 0 && !tap_histo_flag) {
                if (ts == 0)
                    fprintf(fout, "%d:\n", p_tr->fr_nr);
                else
                    fprintf(fout, "%d(%u):\n", p_tr->fr_nr, ts);
            }
            print_tr_diag_bin((uint8_t*)&p_tr->tr[rec_idx]);

            break;  // end of records
        } 

        if (gs_rec_found || (p_tr->tr[rec_idx].bar_id == 0 &&
                             p_tr->tr[rec_idx].trk_id == 0 &&  
                             p_tr->tr[rec_idx].force_lvl >= GS_RPT_TYPE_HDR)) {
            // gesture section
            if (gs_rec_found == 0) {
                p_gs_hdr_rec_t p = (p_gs_hdr_rec_t) &p_tr->tr[rec_idx];
                if (rec_idx == 0 && !tap_histo_flag) {
                    if (ts == 0)
                        fprintf(fout, "%d:\n", p_tr->fr_nr);
                    else
                        fprintf(fout, "%d(%u):\n", p_tr->fr_nr, ts);
                }
                print_gs_hdr_bin(p);
                gs_rec_found = 1;
            } else {
                // Gesture Report
                // First guess is slider.  If that's wrong then keep trying
                p_gs_slider_rec_t p = (p_gs_slider_rec_t) &p_tr->tr[rec_idx];
                if (p->escape0 == GS_RPT_TYPE_SLIDE) {
                    print_gs_slide_bin(p);
                } else if (p->escape0 == GS_RPT_TYPE_SQUEEZE) {
                    print_gs_squeeze_bin((p_gs_squeeze_rec_t)p);
                } else {      
                    // This is not good.  Better to report it than to ignore it
                    fprintf(fout,"    Unknown Gesture Report id (%d)\n", p->escape0);
                    fprintf(fout,"    0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x 0x%02x\n",
                              ((uint8_t*)&p_tr->tr[rec_idx])[0],
                              ((uint8_t*)&p_tr->tr[rec_idx])[1],
                              ((uint8_t*)&p_tr->tr[rec_idx])[2],
                              ((uint8_t*)&p_tr->tr[rec_idx])[3],
                              ((uint8_t*)&p_tr->tr[rec_idx])[4],
                              ((uint8_t*)&p_tr->tr[rec_idx])[5],
                              ((uint8_t*)&p_tr->tr[rec_idx])[6],
                              ((uint8_t*)&p_tr->tr[rec_idx])[7]);
                }
            }

        } else {
            // track reports section
            print_track_rec_bin(p_tr->fr_nr, &p_tr->tr[rec_idx], ts);
        }
        rec_idx++;
    }
}

#endif




void process_tr_diag_bin_file(void)
{
    uint16_t rec_len;
    uint32_t ts;
    bar1d_host_tr_t tr;
    int ret;
    int tr_i = 0;

    while (42) {
        // read log record length
        ret = fread((void*)&rec_len, sizeof(uint16_t), 1, fin);
        if (ret != 1) {
            if (!feof(fin)) 
                fprintf(ferr, "Failed to read bin track report log rec size %d - EOF\n", ret);
            return;
        }

        // read timestamp
        ret = fread((void*)&ts, sizeof(uint32_t), 1, fin);
        if (ret != 1) {
            fprintf(ferr, "Failed to read bin track report log rec timestamp %d\n", ret);
            return;
        }

        // read tr record length
        ret = fread((void*)&tr.length, sizeof(uint16_t), 1, fin);
        if (ret != 1) {
            fprintf(ferr, "Failed to read bin track report rec size %d - EOF\n", ret);
            return;
        }

        // read fr_nr+tr reports
        ret = fread((void*)&tr.fr_nr, sizeof(uint8_t), tr.length, fin);
        if (ret != tr.length) {
            fprintf(ferr, "Failed to read bin track report rec size %d - EOF\n", ret);
            return;
        }

        if (print_level >= PRINT_LEVEL_VERBOSE) {
          fprintf(fout, "\n**********************************************************\n");
          fprintf(fout, "TFIFO Report record index      = %d\n", tr_i++);
          fprintf(fout, "TFIFO Report length            = %d\n", rec_len);
          fprintf(fout, "TFIFO Report timestamp         = %d\n", ts);
          fprintf(fout, "TFIFO Report Content Length    = %d\n", tr.length);
          fprintf(fout, "TFIFO Report Frame Number      = %d\n", tr.fr_nr);
        }

        // print reports
        print_tr_bin(&tr, ts);
    }
}




/*****************************************************************************
 * 
 * LEGACY TRACK REPORT TEXT FILE PROCESSING
 * 
 ****************************************************************************/

/*****************************************************************************
 * process_tr_diag_txt_file_legacy()
 * 
 * This function processes track_report.log file from older driver format.
 * 
 */
void process_tr_diag_txt_file_legacy(void)
{
    int num_token;
    char line[256];
    char *p;
    input_tr_rec_t tr_in;
    int lineno=1;

  
    p = fgets(line, 256, fin);
    while (p) {

        /* snt8100fsr driver adds a \0 after first line. take it out */
        if (line[0]==0) p = line+1; else p = line;


        num_token = sscanf(p,"%u, %u, %u, %u, %u, %u, %u, %u, %u, %u",
                            &tr_in.ts,
                            &tr_in.fn,
                            &tr_in.bar_id,
                            &tr_in.a1,
                            &tr_in.a2,
                            &tr_in.a3,
                            &tr_in.a4,
                            &tr_in.a5,
                            &tr_in.a6,
                            &tr_in.a7);

        if (num_token == 8 || num_token == 10) // piezo or strain report
            process_report_rec(&tr_in);
        else
            fprintf(fout,"%s",line);

        p = fgets(line, 256, fin);
        lineno++;
    }
}


/*****************************************************************************
 * process_report_rec()
 * 
 * Process a legacy line of track_report.log that has been put in 
 * input_tr_rec_t structure.
 * 
 */
void process_report_rec(p_input_tr_rec_t p_in)
{
    /* add extra \n on new record  */
    if (p_in->ts != cur_ts) {
        fprintf(fout, "\n");
    }

    switch (state) {
        case STATE_TR: {
            if (p_in->bar_id == 0 && p_in->a1 == 0) {
                start_tr_diag(p_in);
            } else if (IS_STRAIN_BAR(p_in->bar_id)) {
                print_region_report_from_in_report(p_in);
            } else {
                print_position_report_from_in_report(p_in);
            }

        }
        break;

        case STATE_TR_DIAG: {
            process_tr_diag_legacy(p_in);
        }
        break;

        default: fprintf(ferr, "ERROR: unknown state %d\n", state); exit(1);
    }

    /* update new record indicator */
    if (p_in->ts != cur_ts)
        cur_ts = p_in->ts;
}

/*****************************************************************************
 * start_tr_diag()
 *
 * start building tr diag structure from legacy track_report.log
 * 
 * Initialize state transition to processing tr_diag section of track report.
 * Key opeation is taking track reports and refactoring their data into the
 * tr_diag structure.
 *
 * conversion of normal tr to bytes in tr_diag:
 *
 * bar_id<<3 | a1&0x3 = byte0
 * a2 = byte1
 * a3 = byte2 | byte3
 * a4 = byte4 | byte5
 * a5 = byte6 | byte7
 *
 * conversion of strain tr to bytes in tr_diag:
 *
 * bar_id << 3 = byte0 (WARNING! MISSING 3lsb0:2)
 * a1 = byte1
 * a2 = byte2
 * a3 = byte3
 * a4 = byte4
 * a5 = byte5
 * a6 = byte6
 * a7 = byte7
 *
 ****************************************************************************/

void start_tr_diag(p_input_tr_rec_t p_in)
{
    if (p_in->ts != cur_ts) {
        fprintf(fout, "%u, %u\n", p_in->ts, p_in->fn);
    }
    state = STATE_TR_DIAG;
    tr_cur_rec = 1;

    // currently 2 versions of tr_diag structure defined.
    if (p_in->a2 == 1) {
        p_tr_diag_base = (uint8_t*) &tr_diag_rec_001;
        p_tr_diag_buf = p_tr_diag_base;
        tr_max_rec = TR_DIAG_REC_VERS_001;
    } else if (p_in->a2 == 2) {
        p_tr_diag_base = (uint8_t*) &tr_diag_rec_002;
        p_tr_diag_buf = p_tr_diag_base;
        tr_max_rec = TR_DIAG_REC_VERS_002;
    } else {
        fprintf(ferr, "ERROR Unkown tr_diag version %d\n", (int)p_in->a2);
        exit(1);
    }
    *p_tr_diag_buf++ = 0; // barid|a1 is 0 by def
    *p_tr_diag_buf++ = p_in->a2;
    *p_tr_diag_buf++ = p_in->a3&0xff;
    *p_tr_diag_buf++ = (p_in->a3>>8)&0xff;
    *p_tr_diag_buf++ = p_in->a4&0xff;
    *p_tr_diag_buf++ = (p_in->a4>>8)&0xff;
    *p_tr_diag_buf++ = p_in->a5&0xff;
    *p_tr_diag_buf++ = (p_in->a5>>8)&0xff;

    if (tr_cur_rec == tr_max_rec) {
        print_tr_diag_bin(p_tr_diag_base);
        state = STATE_TR;
    }
}


/*****************************************************************************
 * process_tr_diag_legacy()
 *
 * continue converting track report records into the tr_diag structure.
 *
 ****************************************************************************/
void process_tr_diag_legacy(p_input_tr_rec_t p_in)
{
    if (IS_STRAIN_BAR(p_in->bar_id)) {
        fprintf(ferr, "WARNING! Possible loss of data in tr_diag fr_nr=%u\n",p_in->fn);
        *p_tr_diag_buf++ = p_in->bar_id<<3;
        *p_tr_diag_buf++ = p_in->a1;
        *p_tr_diag_buf++ = p_in->a2;
        *p_tr_diag_buf++ = p_in->a3;
        *p_tr_diag_buf++ = p_in->a4;
        *p_tr_diag_buf++ = p_in->a5;
        *p_tr_diag_buf++ = p_in->a6;
        *p_tr_diag_buf++ = p_in->a7;

    } else {
        *p_tr_diag_buf++ = p_in->bar_id<<3 | p_in->a1&0x3;
        *p_tr_diag_buf++ = p_in->a2;
        *p_tr_diag_buf++ = p_in->a3&0xff;
        *p_tr_diag_buf++ = (p_in->a3>>8)&0xff;
        *p_tr_diag_buf++ = p_in->a4&0xff;
        *p_tr_diag_buf++ = (p_in->a4>>8)&0xff;
        *p_tr_diag_buf++ = p_in->a5&0xff;
        *p_tr_diag_buf++ = (p_in->a5>>8)&0xff;
    }
    tr_cur_rec++;
    if (tr_cur_rec == tr_max_rec) {
        print_tr_diag_bin(p_tr_diag_base);
        state = STATE_TR;
    }
}


/*****************************************************************************
 * print_region_report_from_in_report()
 * 
 * Print region track report from text "in" report  
 *
 ****************************************************************************/
void print_region_report_from_in_report(p_input_tr_rec_t p_in)
{
    stg_track_report_t t;
    if (p_in){
        t.bar_id = p_in->bar_id;
        t.force_lvl[0] = p_in->a1;
        t.force_lvl[1] = p_in->a2;
        t.force_lvl[2] = p_in->a3;
        t.force_lvl[3] = p_in->a4;
        t.force_lvl[4] = p_in->a5;
        t.force_lvl[5] = p_in->a6;
        t.force_lvl[6] = p_in->a7;
        print_region_report_bin(p_in->ts, p_in->fn, &t);
    }
}


/*****************************************************************************
 * print_position_report_from_in_report()
 * 
 * Print a positional track report from the txt "in" record
 *
 ****************************************************************************/
void print_position_report_from_in_report(p_input_tr_rec_t p_in)
{
    if (p_in) {
        track_report_t t;
        t.bar_id = (uint8_t)p_in->bar_id;
        t.trk_id = (uint8_t)p_in->a1;
        t.force_lvl = (uint8_t)p_in->a2;
        t.center = (uint16_t)p_in->a3;
        t.bottom = (uint16_t)p_in->a4;
        t.top = (uint16_t)p_in->a5;
        print_position_report_bin(p_in->ts, p_in->fn,  &t);
    }
}

/*****************************************************************************
 * 
 * TRACK REPORT TEXT FILE PROCESSING
 * 
 ****************************************************************************/

/*****************************************************************************
 * process_tr_diag_txt()
 *
 * Process one or more lines of input from track_report.log into a tr diag 
 * structure and then print it out. 
 * 
 */
void process_tr_diag_txt(char *p)
{
    uint8_t *b = p_tr_diag_buf;

    if (state == STATE_TR) {

        // Start of tr diag struct so reset pointers.
        p_tr_diag_buf = (uint8_t*) tr_diag_build_txt;
        b = p_tr_diag_buf;
        uint32_t ts, fn;

        // Initialize buffer pointers and search for start of tr diag section
        // start will have ts, fn, 0xbyte0, .. byte7

        if (sscanf(p,"%u, %u, 0x%hhx, %hhu, %hhu, %hhu, %hhu, %hhu, %hhu, %hhu",
            &ts,&fn,&b[0],&b[1],&b[2],&b[3],&b[4],&b[5],&b[6],&b[7])==10) {

            state = STATE_TR_DIAG;
            p_tr_diag_buf += 8/sizeof(*p_tr_diag_buf);

            // print banner for the tr diag record
            if (ts == 0)
                fprintf(fout, "%d:\n", fn);
            else
                fprintf(fout, "%d(%u):\n", fn, ts);

            } else {
                fprintf(fout, "Unexpected tr diag start %s\n", p);
                exit(1);
            }

    } else if (state == STATE_TR_DIAG) {

        // check for eof, call print function and reset state if found
        if (strcmp(p,"\n")==0) {
            print_tr_diag_bin((uint8_t*)tr_diag_build_txt);
            state = STATE_TR;

        // check for tr diag continuation record of 8 bytes
        } else if (sscanf(p,"%hhu, %hhu, %hhu, %hhu,%hhu, %hhu, %hhu, %hhu",
                   &b[0],&b[1],&b[2],&b[3],&b[4],&b[5],&b[6],&b[7])==8) {
            p_tr_diag_buf += 8/sizeof(*p_tr_diag_buf);
        } else {
            fprintf(fout, "Unexpected tr diag line %s\n",p);
            exit(1);
        }

    } else {
        fprintf(fout, "ERROR Unknown state %d\n", state);
        exit(1);
    }
}

/*****************************************************************************
 * process_escape_rec_txt()
 * 
 * Process a gesture or tr diag section from track_report.log
 * 
 */
void process_escape_rec_txt(char *p)
{
    input_tr_rec_t tr_in;
    uint32_t escape, ts, fn;
    int num_token;

    num_token = sscanf(p, "%u, %u, 0x%x",&ts, &fn, &escape);
    if (num_token==3 || state == STATE_TR_DIAG) {
        if (escape == 0 || state == STATE_TR_DIAG) {
            //fprintf(fout, "tr diag: %s\n", p);
            process_tr_diag_txt(p);

        } else if (escape == GS_RPT_TYPE_HDR) {
            //fprintf(fout, "gesture hdr: %s\n", p);
                if (ts == 0)
                fprintf(fout, "%d:\n", fn);
            else
                fprintf(fout, "%d(%u):\n", fn, ts);

            print_gs_hdr_txt(p);

        } else if (escape == GS_RPT_TYPE_SLIDE) {
            //fprintf(fout, "gesture slide: %s\n", p);
            print_gs_slide_txt(p);

        } else if (escape == GS_RPT_TYPE_SQUEEZE) {
            //fprintf(fout, "gesture squeeze: %s\n", p);
            print_gs_squeeze_txt(p);

        } else  {
            fprintf(fout, "unknown: %s\n", p);
            // unknown type
        }

    } else {
        // num_token wrong
        fprintf(fout, "UNKNOWN Gesture %s\n", p);
    }
}

/*****************************************************************************
 * process_tr_diag_txt_file()
 * 
 * This function processes track_report.log file.
 * 
 */
void process_tr_diag_txt_file(void)
{
    int num_token;
    char line[256];
    char *p;
    input_tr_rec_t tr_in;
    int lineno=1;

  
    p = fgets(line, 256, fin);
    while (p) {
         uint32_t escape, ts, fn;
        /* snt8100fsr driver adds a \0 after first line. take it out */
        if (line[0]==0) p = line+1; else p = line;

        num_token = sscanf(p, "%u, %u, 0x%x",&ts, &fn, &escape);
        if (num_token==3 || state == STATE_TR_DIAG) {
            process_escape_rec_txt(p);

        } else { // handle normal track reports

            num_token = sscanf(p,"%u, %u, %u, %u, %u, %u, %u, %u, %u, %u",
                                &tr_in.ts,
                                &tr_in.fn,
                                &tr_in.bar_id,
                                &tr_in.a1,
                                &tr_in.a2,
                                &tr_in.a3,
                                &tr_in.a4,
                                &tr_in.a5,
                                &tr_in.a6,
                                &tr_in.a7);

            if (num_token == 8 || num_token == 10) // piezo or strain report
                process_report_rec(&tr_in);
            else
                fprintf(fout,"%s",line);
        }

        p = fgets(line, 256, fin);
        lineno++;
    }
}


/*****************************************************************************
 * print_gs_hdr_txt()
 * 
 * convert gesture header from track_report.log to binary format and print it.
 * 
 */
void print_gs_hdr_txt(char *p)
{
     // ts, fn, 0x80, sw0, sw1, sw2, sq_st, sq_cn, sq_sh, sq_ln, sq_en, 
     //               ts0, ts1, ts2, ts3, tp0, tp1, tp2, tp3
     //
    input_tr_rec_t tr_in;
    int num_token = sscanf(p,"%u, %u, 0x%x, %d, %d, %d, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u, %u",
                                &tr_in.ts,
                                &tr_in.fn,
                                &tr_in.bar_id,
                               (int*) &tr_in.a1,    // sw0
                               (int*) &tr_in.a2,    // sw1
                                (int*)&tr_in.a3,    // sw2
                                &tr_in.a4,          // sq_st
                                &tr_in.a5,          // sq_cn
                                &tr_in.a6,          // sq_sh
                                &tr_in.a7,          // sq_ln
                                &tr_in.a8,          // sq_en
                                &tr_in.a9,          // ts0
                                &tr_in.a10,         // ts1
                                &tr_in.a11,         // ts2
                                &tr_in.a12,         // ts3
                                &tr_in.a13,         // tp0
                                &tr_in.a14,         // tp1
                                &tr_in.a15,         // tp2
                                &tr_in.a16,         // tp3
                                &tr_in.a17,         // dts0
                                &tr_in.a18,         // dts1
                                &tr_in.a19,         // dts2
                                &tr_in.a20,         // dtp0
                                &tr_in.a21,         // dtp1
                                &tr_in.a22          // dtp2
                                );
    if (num_token != 19 || tr_in.bar_id != GS_RPT_TYPE_HDR) {
        fprintf(stderr, "ERROR: Expected Gesture Hdr\n");
        exit(1);
    }

    gs_hdr_rec_t h;
    h.escape0 = 0;
    h.escape1 = GS_RPT_TYPE_HDR;
    h.swipe0_velocity = (int)tr_in.a1;
    h.swipe1_velocity = (int)tr_in.a2;
    h.swipe2_velocity = (int)tr_in.a3;
    h.tap = GS_SET_TAP_START(0,0,tr_in.a9)   |
            GS_SET_TAP_START(0,1,tr_in.a10)  |
            GS_SET_TAP_START(0,2,tr_in.a11)  |
            GS_SET_TAP_START(0,3,tr_in.a12)  |
            GS_SET_TAP_STOP(0,0,tr_in.a13)   |
            GS_SET_TAP_STOP(0,1,tr_in.a14)   |
            GS_SET_TAP_STOP(0,2,tr_in.a15)   |
            GS_SET_TAP_STOP(0,3,tr_in.a16);
    h.squeeze = 
            GS_PUT_SQUEEZE_START(tr_in.a4)   |
            GS_PUT_SQUEEZE_CANCEL(tr_in.a5)  |
            GS_PUT_SQUEEZE_SHORT(tr_in.a6)   |
            GS_PUT_SQUEEZE_LONG(tr_in.a7)    |
            GS_PUT_SQUEEZE_END(tr_in.a8);
    h.dtap= GS_SET_TAP_START(0,0,tr_in.a17)  |
            GS_SET_TAP_START(0,1,tr_in.a18)  |
            GS_SET_TAP_START(0,2,tr_in.a19)  |
            GS_SET_TAP_STOP(0,0,tr_in.a20)   |
            GS_SET_TAP_STOP(0,1,tr_in.a21)   |
            GS_SET_TAP_STOP(0,2,tr_in.a22);

    print_gs_hdr_bin(&h);
    
}

void print_gs_slide_txt(char *p)
{
    // ts, fn, 0x81, id, fid, frc, pos
    //
    input_tr_rec_t tr_in;
    int num_token = sscanf(p,"%u, %u, 0x%x, %u, %u, %u, %u",
                                &tr_in.ts,
                                &tr_in.fn,
                                &tr_in.bar_id,
                                &tr_in.a1,          // id
                                &tr_in.a2,          // fid
                                &tr_in.a3,          // frc
                                &tr_in.a4);         // pos
    if (num_token != 7 || tr_in.bar_id != GS_RPT_TYPE_SLIDE) {
        fprintf(stderr, "ERROR: Expected Gesture Slide\n");
        exit(1);
    }
    gs_slider_rec_t s;
    s.escape0 = GS_RPT_TYPE_SLIDE;
    s.slider_finger_id = GS_PUT_SLIDER_ID0(tr_in.a1)|GS_PUT_SLIDER_FID0(tr_in.a2);
    s.slider_force0 = tr_in.a3;
    s.slider_force1 = 0;
    s.slider_pos0   = tr_in.a4;
    s.slider_pos1   = 0;
    print_gs_slide_bin(&s);
}

void print_gs_squeeze_txt(char *p)
{
    // ts, fn, 0x82, sq_idx, sq_st, sq_cn, sq_sh, sq_ln, sq_en
    //
    input_tr_rec_t tr_in;
    int num_token = sscanf(p,"%u, %u, 0x%x, %u, %u, %u, %u, %u, %u",
                                &tr_in.ts,
                                &tr_in.fn,
                                &tr_in.bar_id,
                                &tr_in.a1,          // idx
                                &tr_in.a2,          // sq_st
                                &tr_in.a3,          // sq_cn
                                &tr_in.a4,          // sq_sh
                                &tr_in.a5,          // sq_ln
                                &tr_in.a6);         // sq_en
    if (num_token != 9 || tr_in.bar_id != GS_RPT_TYPE_SQUEEZE) {
        fprintf(stderr, "ERROR: Expected Gesture Squeeze\n");
        exit(1);
    }
    gs_squeeze_rec_t s;
    memset(&s, 0, sizeof(s));
    s.escape0 = GS_RPT_TYPE_SQUEEZE;
    s.squeeze[tr_in.a1] = 
            GS_PUT_SQUEEZE_START(tr_in.a2)   |
            GS_PUT_SQUEEZE_CANCEL(tr_in.a3)  |
            GS_PUT_SQUEEZE_SHORT(tr_in.a4)   |
            GS_PUT_SQUEEZE_LONG(tr_in.a5)    |
            GS_PUT_SQUEEZE_END(tr_in.a6);
    print_gs_squeeze_bin(&s);
}



/*****************************************************************************
 * 
 * EVENT LOG PROCESSING
 * 
 ****************************************************************************/



void process_event_log_bin_file(void)
{
    uint32_t ev[2];
    int ret;
    uint32_t status;

    ret = fread((void*)&status, sizeof(uint8_t), sizeof(status), fin);
    if (ret != sizeof(status)) {
        if (!feof(fin)) 
            fprintf(ferr, "Failed to read event log status size %d\n", ret);
        return;
    }
    fprintf(fout,"EventLog Status = %d\n", status);

    while (42) {
        // read log record 
        ret = fread((void*)&ev, sizeof(uint8_t), sizeof(ev), fin);
        if (ret != sizeof(ev)) {
            if (!feof(fin)) 
                fprintf(ferr, "Failed to read event log rec size %d\n", ret);
            return;
        }
        fprintf(fout,"0x%02x  0x%02x  0x%04x  0x%04x  0x%04x\n",
                ev[0] >> 24 & 0xff,
                ev[0] >> 16 & 0xff,
                ev[0] & 0xffff,
                ev[1] >> 16 & 0xffff,
                ev[1] & 0xffff);
    }

}

/*****************************************************************************
 * 
 * DEEP TRACE PROCESSING
 * 
 ****************************************************************************/


void process_deep_trace_file(void)
{
    int i;
    uint16_t trace_len;
    uint16_t r_idx;
    uint16_t w_idx;
    int ret = fread((void*)&trace_len, sizeof(uint16_t), 1, fin);
    if (ret != 1) {
        fprintf(ferr, "Failed to read deep trace size %d\n", ret);
        return;
    }
    ret = fread((void*)&r_idx, sizeof(uint16_t), 1, fin);
    if (ret != 1) {
        fprintf(ferr, "Failed to read deep trace r_idx size\n");
        return;
    }
    ret = fread((void*)&w_idx, sizeof(uint16_t), 1, fin);
    if (ret != 1) {
        fprintf(ferr, "Failed to read deep trace w_idx size\n");
        return;
    }
    trace_len -= 2*sizeof(uint16_t);

    uint8_t *p_buf = malloc(trace_len);
    if (p_buf==NULL) {
        fprintf(ferr,"Could not malloc deep trace buffer %d\n", trace_len);
        return;
    }

    ret = fread((void*)p_buf, sizeof(uint8_t), trace_len, fin);
    if (ret != trace_len) {
        fprintf(ferr, "Could not read full deep trace file (%d!=%d)\n", ret, trace_len);
        return;
    }

    while (r_idx != w_idx) {
        uint8_t rec_len = p_buf[r_idx] - sizeof(uint8_t);     // sub out len field
        r_idx = (r_idx+1) % trace_len;
        bar1d_host_tr_t tr;
        uint8_t *p_tr_buf = (uint8_t*)&tr;
        for (i=0; i < rec_len; i++) {
            p_tr_buf[i] = p_buf[r_idx];
            r_idx = (r_idx+1) % trace_len;
        }
        print_tr_bin(&tr, 0 /*no ts*/);
    }
    
}

// deep trace size is based on this structure:
//
// (16*1024+4)* 3 = 49164
//
// buf in deep trace structure is: 49164 - 3*sizeof(uint16) = 49158
#define DEEP_TRACE_TOTAL_SIZE ((16*1024 + sizeof(uint32_t)) * 3)
#define DEEP_TRACE_BUF_SIZE   (DEEP_TRACE_TOTAL_SIZE - 3*sizeof(uint16_t))

  struct deep_trace_s
  {
    uint16_t length;
    uint16_t r_idx;
    uint16_t w_idx;
    uint8_t  tbuf[DEEP_TRACE_BUF_SIZE];

  } deep_trace;


void strip_gs_diag_from_tr(p_bar1d_host_tr_t p_tr)
{
  if (!p_tr || p_tr->length <= 2) return;

  // scan through track reports unilt we find one whose first byte is 0
  // This is the start of the gesture and/or tr-diag section so just
  // set length to only include the track reports and return.
  int num_rec = (p_tr->length - sizeof(uint16_t))/sizeof(track_report_t);
  for (int i=0; i < num_rec; i++) {
    if (p_tr->tr[i].bar_id==0 && p_tr->tr[i].trk_id==0) { // start of gest/diag 
      // i == number of non-gs, non-diag track reports.
      p_tr->length = sizeof(uint16_t) + sizeof(track_report_t)*i;
      return;
    }
  }
 
}

void convert_tr_bin_2_deep_trace(void)
{
    // initialize deep_trace. length doesn't include itself as it conforms to 
    // FIFO length field policies. r_idx should be 0.
    memset((void*)&deep_trace, 0, sizeof(deep_trace));
    deep_trace.length = sizeof(deep_trace) - sizeof(uint16_t); 

    int bytes_read;
    uint16_t rec_len;
    uint32_t ts;
    bar1d_host_tr_t tr;
    int tr_i = 0;
    int overflow = 0;

    while (42) {
        // read log record length
        bytes_read = fread((void*)&rec_len, sizeof(uint16_t), 1, fin);
        if (bytes_read != 1) {
            if (!feof(fin)) 
                fprintf(ferr, "Failed to read bin track report log rec size %d - EOF\n", bytes_read);
            break;
        }
        tr_i++;

        // read timestamp
        bytes_read = fread((void*)&ts, sizeof(uint32_t), 1, fin);
        if (bytes_read != 1) {
            fprintf(ferr, "Failed to read bin track report log rec timestamp %d\n", bytes_read);
            return;
        }

        // read tr record length
        bytes_read = fread((void*)&tr.length, sizeof(uint16_t), 1, fin);
        if (bytes_read != 1) {
            fprintf(ferr, "Failed to read bin track report rec size %d - EOF\n", bytes_read);
            return;
        }

        // read fr_nr+tr reports
        bytes_read = fread((void*)&tr.fr_nr, sizeof(uint8_t), tr.length, fin);
        if (bytes_read != tr.length) {
            fprintf(ferr, "Failed to read bin track report rec size %d - EOF\n", bytes_read);
            return;
        }

        if (print_level >= PRINT_LEVEL_VERBOSE) {
          fprintf(fout, "\n**********************************************************\n");
          fprintf(fout, "TFIFO Report record number      = %d\n", tr_i);
          fprintf(fout, "TFIFO Report length            = %d\n", rec_len);
          fprintf(fout, "TFIFO Report timestamp         = %d\n", ts);
          fprintf(fout, "TFIFO Report Content Length    = %d\n", tr.length);
          fprintf(fout, "TFIFO Report Frame Number      = %d\n", tr.fr_nr);
        }

        if (!overflow) {
          // strip out gesture and tr diag records from tr
          strip_gs_diag_from_tr(&tr);

          // verify there is enough room in deep trace for this record. Total
          // record size is tr length + tr.length field + one byte for deep 
          // trace length field.
          rec_len = tr.length + sizeof(uint16_t) + sizeof(uint8_t);
          if (deep_trace.w_idx + rec_len >= DEEP_TRACE_BUF_SIZE) {
              fprintf(ferr, "WARNING: deep trace coversion truncating at track report %d\n", tr_i);
              overflow++;
          } else {

            // Add it to deep trace. Note we don't have to worry about wraparound
            // since we stop when its full.
            deep_trace.tbuf[deep_trace.w_idx++] = rec_len;
            int size_to_copy =  tr.length + sizeof(uint16_t);
            memcpy((void*)&deep_trace.tbuf[deep_trace.w_idx],(void*)&tr, size_to_copy);
            deep_trace.w_idx += size_to_copy;
          }
        }
    }
    fprintf(ferr,"Number of TFIFO Reports added to %s = %d\n", out_fname, tr_i);
    bytes_read = fwrite((void*)&deep_trace, sizeof(uint8_t), sizeof(deep_trace), fout);
    if (bytes_read != sizeof(deep_trace)) {
      fprintf(ferr, "ERROR trying to write deep_trace record to %s, only %d bytes written\n", 
                out_fname, bytes_read);
    }
}


/* EOF */


