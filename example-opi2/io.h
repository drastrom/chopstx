DEFINE_HW (PLL_CPUX, static, 0x01c20000) {
  volatile uint32_t CTRL;
};
DEFINE_HW (PLL_DDR, static, 0x01c20020) {
  volatile uint32_t CTRL;
};
DEFINE_HW (PLL_PERIPH0, static, 0x01c20028) {
  volatile uint32_t CTRL;
};
DEFINE_HW (CPUX_AXI_CFG, static, 0x01c20050) {
  volatile uint32_t REG;
};
DEFINE_HW (AHB1_APB1_CFG, static, 0x01c20054) {
  volatile uint32_t REG;
};

DEFINE_HW (BUS_CLK_GATING, static, 0x01c20060) {
  volatile uint32_t REG0;
  volatile uint32_t REG1;
  volatile uint32_t REG2;
  volatile uint32_t REG3;
  volatile uint32_t REG4;
};

DEFINE_HW (DRAM_CONFIG, static, 0x01c200f4) {
  volatile uint32_t REG;
};

DEFINE_HW (MBUS_RST, static, 0x01c200fc) {
  volatile uint32_t REG;
};

DEFINE_HW (MBUS_CLK, static, 0x01c2015c) {
  volatile uint32_t REG0;
  volatile uint32_t REG1;
};

DEFINE_HW (DDR_TUNE, static, 0x01c20260) {
  volatile uint32_t REG;
};

DEFINE_HW (DDR_PAT, static, 0x01c20290) {
  volatile uint32_t CTRL;
};

DEFINE_HW (BUS_SOFT_RST, static, 0x01c202c0) {
  volatile uint32_t REG0;
  volatile uint32_t REG1;
};

struct PORT {
  volatile uint32_t CFG0;	/* Port 0-7 */
  volatile uint32_t CFG1;	/* Port 8-15 */
  volatile uint32_t CFG2;	/* Port 16-23 */
  volatile uint32_t CFG3;	/* Port 24-31 */
  volatile uint32_t DATA;
  volatile uint32_t LEVEL0;
  volatile uint32_t LEVEL1;
  volatile uint32_t PULL0;
  volatile uint32_t PULL1;
};

static struct PORT *const PORTA = (struct PORT *const)0x01c20800;
static struct PORT *const PORTL = (struct PORT *const)0x01f02c00;

DEFINE_HW (RPRCM_GATING, static, 0x01f01428) {
  volatile uint32_t REG;
};



static inline void
delay (uint32_t d)
{
  while (d--)
    asm volatile ("" : : : "memory");
}
