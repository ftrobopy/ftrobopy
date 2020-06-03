# origin of folders folders "buildroot" and "txt_demo_c_download" is https://github.com/fischertechnik

/home/ft/software/FT/u-boot/gcc-linaro-arm-linux-gnueabihf-4.9-2014.05_linux/bin/arm-linux-gnueabihf-g++ -shared -fPIC \
-I/home/ft/software/FT/buildroot/output/build/python3-3.6.3/Include \
-I/home/ft/software/FT/buildroot/output/build/python3-3.6.3 \
-I/home/ft/software/FT/txt_demo_c_download/deps/include \
-I/home/ft/software/FT/buildroot/output/build/sdl-1.2.15/include \
-L/home/ft/software/FT/buildroot/output/target/usr/lib \
-lpython4 -lKeLibTxt -lMotorIOLib -lTxtControlLib -lROBOProLib -lSDL -lSDL_ttf -lSDL_gfx -lSDL_image \
ftTA2py.c -o ftTA2py.so

