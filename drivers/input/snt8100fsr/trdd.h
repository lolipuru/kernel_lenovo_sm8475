/*****************************************************************************
 * FUNCTION DECLARATIONS
 ****************************************************************************/

void process_report_rec(p_input_tr_rec_t p_in);
void start_tr_diag(p_input_tr_rec_t p_in);
void process_tr_diag_legacy(p_input_tr_rec_t p_in);
void print_region_report_from_in_report(p_input_tr_rec_t p_in);
void print_position_report_from_in_report(p_input_tr_rec_t p_in);
void print_tr_diag_bin(uint8_t *p_buf);
void print_tr_diag_001(void *p_buf);
void print_tr_diag_002(void *p_buf);
void print_tr_diag_003(void *p_buf);
void print_tr_diag_004(void *p_buf);
void print_tr_diag_005(void *p_buf);
void process_tr_diag_txt_file(void);
void process_tr_diag_txt_file_legacy(void);
void process_tr_diag_bin_file(void);
void process_deep_trace_file(void);
void process_event_log_bin_file(void);
void print_gs_hdr_txt(char *p);
void print_gs_slide_txt(char *p);
void print_gs_squeeze_txt(char *p);
void print_gs_hdr_bin(p_gs_hdr_rec_t p);
void process_tr_diag_txt(char *p);

