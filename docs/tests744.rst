Hardware readme
##################################
Feb,11, 2019


sf read 0x25000000 0x4D0000 0xC00000: failed; less than it, OK!

setenv bootcmd "sf probe 1:0; sf read 0x22000000 0xB0000 0x400000; sf read 0x25000000 0x4D0000 0x1000000; sf read 0x21000000 0xA0000 0x10000"

setenv bootcmd "sf probe 1:0; sf read 0x22000000 0xB0000 0x400000; sf read 0x25000000 0x4D0000 0x1000000; sf read 0x21000000 0xA0000 0x10000;;bootz 0x22000000 - 0x21000000"


sf read 0x25000000 0x3D0000 0x1000000

sf read 0x22000000 0x3D0000 0x1000000

In uboot:

   ping 192.168.168.1

   i2c md 30 0 30: read 30 bytes from offset of 0
   

   
   0x0000014d0000-0x0000018d0000
   
   sf probe 0
   sf erase 0x14d0000 0x400000
   sf write 0x21000000 0x14d0000 0x400000
   
   mkfs.jffs2 -r ./txapp0816 -e 0x10000 -p 0x400000 -n -l -o jffs2_txapp

   mkfs.jffs2 -r ./rxapp0813 -e 0x10000 -p 0x400000 -n -l -o jffs2_rxapp

//erase app mtd  4M

   flash_erase /dev/mtd8 0 64

   flashcp txapp /dev/mtd8

   flashcp rxapp /dev/mtd8

   
   
   