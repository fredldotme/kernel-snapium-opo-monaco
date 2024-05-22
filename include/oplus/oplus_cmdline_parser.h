#ifndef __OPLUS_CMDLINE_PARSER_H
#define __OPLUS_CMDLINE_PARSER_H




unsigned int get_oplus_hw_id(void);
unsigned int get_oplus_board_type(void);
unsigned int get_oplus_panel_id(void);
unsigned int get_panel_present(void);
unsigned int get_dsi_relock_en(void);
unsigned int get_user_printk_disable_uart(void);
unsigned int get_display_vflip(void);

bool is_user_build(void);
bool is_recovery(void);

#endif /* __OPLUS_CMDLINE_PARSER_H */
