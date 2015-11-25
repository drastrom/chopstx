/*
 * Macros for hardware registers
 */
#if 0
QUALIFIER struct NAME *const NAME = (struct NAME *const)ADDRESS;
struct NAME {
  entry0;
  entry1;
  entry2;
  ...
};
#endif

#define DEFINE_HW(hardware_cmp_name, qualifier, start_address)     \
  qualifier struct hardware_cmp_name *const hardware_cmp_name = \
    (struct hardware_cmp_name *const)start_address;             \
  struct hardware_cmp_name
