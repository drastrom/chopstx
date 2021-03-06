#define BOARD_ID_ST_NUCLEO_L432    0x3a8d5116

extern const uint8_t sys_version[8];
#if defined(USE_SYS3) || defined(USE_SYS_BOARD_ID)
extern const uint32_t sys_board_id;
extern const uint8_t sys_board_name[];
# define SYS_BOARD_ID sys_board_id
#else
# define SYS_BOARD_ID BOARD_ID
#endif

/* XXX: unique_device_id */

void set_led (int on);

uintptr_t flash_init (const char *f_name);
void flash_unlock (void);
int flash_program_halfword (uintptr_t addr, uint16_t data);
int flash_erase_page (uintptr_t addr);
int flash_check_blank (const uint8_t *p_start, size_t size);
int flash_write (uintptr_t dst_addr, const uint8_t *src, size_t len);
int flash_protect (void);
void __attribute__((noreturn))
flash_erase_all_and_exec (void (*entry)(void));

void usb_lld_sys_init (void);
void usb_lld_sys_shutdown (void);

void __attribute__((noreturn))
nvic_system_reset (void);

void clock_init (void);
void gpio_init (void);
