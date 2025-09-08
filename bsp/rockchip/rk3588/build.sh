# find packages  -name "libmicroros.a" -exec cp -raf ./libmicroros.a   {} \;
# scons -c
scons
cp rtthread.bin /mnt/f/file/rk3588/bishe/
# cp rtthread.bin ~/tftp