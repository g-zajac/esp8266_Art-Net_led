
typedef struct _DD {
        const int esp_chip_id;
        const int id;
} device_details ;

// current
const device_details devices[] = {
  {0x00147593, 241},
  {0x0029DEAD, 242},
  {0x0029DCAB, 243},
  {0x00D2F1AE, 244},
  {0x0000AEDD, 245},
  {0x0029DEF8, 246},
  {0x0000A7EB, 247},
  {0x00010D63, 248},
  {0x00010DC0, 249},
  {0, 240} // default if not defined above
};
