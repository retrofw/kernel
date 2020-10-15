CC := mipsel-linux-gcc
DESTDIR := /home/zwang

TARGET := fb_fill \
	fb_info \
	fb_v_colorbar \
	fb_h_colorbar \
	lcd_ot_ctrl \
	tve_pal_ot_ctrl \
	tve_ntsc_ot_ctrl

.PHONY: all install clean

%.o : %.c
	$(CC) $(CFLAGS) $(CWARN) $(CDEFS) $(CINCLUDE) -c -o $@ $<

all: $(TARGET)

install: $(TARGET)
	cp -av $(TARGET) $(DESTDIR)

clean:
	rm -rf $(TARGET)
