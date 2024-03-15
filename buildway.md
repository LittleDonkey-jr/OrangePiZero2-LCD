# OrangepiZero2 with ST7789v3 1.47 Inch LCD  

## orangepi-build/kernel/orange-pi-5.13-sunxi64/drivers/staging/fbtft/fb_st7789v.c
### reg setup  
```
static int init_display(struct fbtft_par *par)
{
	/* turn off sleep mode */
	write_reg(par, MIPI_DCS_EXIT_SLEEP_MODE);
	mdelay(120);
	write_reg(par, 0x3A,0x05);//65k mode
	write_reg(par, 0x36,0x70);//屏幕显示方向设置(0x36,0x00)水平
	write_reg(par, 0xB2,0x0C,0X0C,0X00,0X33,0X33);//帧率设置
	write_reg(par, 0xB7,0x35);
	write_reg(par, 0xBB,0x35);
	write_reg(par, 0xC0,0x2C);
	write_reg(par, 0xC2,0x01);
	write_reg(par, 0xC3,0x13);
	write_reg(par, 0xC4,0x20);	
	write_reg(par, 0xC6,0x0F);
	write_reg(par, 0xD0,0xA4,0XA1);
	write_reg(par, 0xD6,0xA1);
	write_reg(par,0xE0,0xF0,0x00,0x04,0x04,0x04,0x05,0x29,0x33,0x3E,0x38,0x12,0x12,0x28,0x30);
	write_reg(par,0xE1,0xF0,0x07,0x0A,0x0D,0x0B,0x07,0x28,0x33,0x3E,0x36,0x14,0x14,0x29,0x32);
	write_reg(par,0x21);
	write_reg(par,0x11);
	mdelay(120);
	write_reg(par,0x29);
	return 0;
}
```
### set width and height
```
static struct fbtft_display display = {
	.regwidth = 8,
	.width = 172,
	.height = 320,
	.gamma_num = 2,
	.gamma_len = 14,
	.gamma = DEFAULT_GAMMA,
	.fbtftops = {
		.init_display = init_display,
		.set_var = set_var,
		.set_gamma = set_gamma,
		.blank = blank,
	},
};
```

## orangepi-build/kernel/orange-pi-5.13-sunxi64/drivers/staging/fbtft/fbtft-core.c  

### add include file compare with kernel version 5.13  
```
#include <linux/gpio.h>
#include <linux/of_gpio.h>
```
### modify funtions below 
```
static int fbtft_request_one_gpio(struct fbtft_par *par,
                  const char *name, int index,
                  struct gpio_desc **gpiop)
{
    struct device *dev = par->info->device;
    struct device_node *node = dev->of_node;
    int gpio, flags, ret = 0;
    enum of_gpio_flags of_flags;

    if (of_find_property(node, name, NULL)) {
        gpio = of_get_named_gpio_flags(node, name, index, &of_flags);
        if (gpio == -ENOENT)
            return 0;
        if (gpio == -EPROBE_DEFER)
            return gpio;
        if (gpio < 0) {
            dev_err(dev,
                "failed to get '%s' from DT\n", name);
            return gpio;
        }

         //active low translates to initially low 
        flags = (of_flags & OF_GPIO_ACTIVE_LOW) ? GPIOF_OUT_INIT_LOW :
                            GPIOF_OUT_INIT_HIGH;
        ret = devm_gpio_request_one(dev, gpio, flags,
                        dev->driver->name);
        if (ret) {
            dev_err(dev,
                "gpio_request_one('%s'=%d) failed with %d\n",
                name, gpio, ret);
            return ret;
        }

        *gpiop = gpio_to_desc(gpio);
        fbtft_par_dbg(DEBUG_REQUEST_GPIOS, par, "%s: '%s' = GPIO%d\n",
                            __func__, name, gpio);
    }

    return ret;
}

static int fbtft_request_gpios(struct fbtft_par *par)
{
	int i;
	int ret;

	ret = fbtft_request_one_gpio(par, "reset", 0, &par->gpio.reset);
	if (ret)
		return ret;
	ret = fbtft_request_one_gpio(par, "dc", 0, &par->gpio.dc);
	if (ret)
		return ret;
	ret = fbtft_request_one_gpio(par, "rd", 0, &par->gpio.rd);
	if (ret)
		return ret;
	ret = fbtft_request_one_gpio(par, "wr", 0, &par->gpio.wr);
	if (ret)
		return ret;
	ret = fbtft_request_one_gpio(par, "cs", 0, &par->gpio.cs);
	if (ret)
		return ret;
	ret = fbtft_request_one_gpio(par, "latch", 0, &par->gpio.latch);
	if (ret)
		return ret;
	for (i = 0; i < 16; i++) {
		ret = fbtft_request_one_gpio(par, "db", i,
					     &par->gpio.db[i]);
		if (ret)
			return ret;
		ret = fbtft_request_one_gpio(par, "led", i,
					     &par->gpio.led[i]);
		if (ret)
			return ret;
		ret = fbtft_request_one_gpio(par, "aux", i,
					     &par->gpio.aux[i]);
		if (ret)
			return ret;
	}

	return 0;
}
```


```
static void fbtft_reset(struct fbtft_par *par)
{
    if (!par->gpio.reset)
        return;
    fbtft_par_dbg(DEBUG_RESET, par, "%s()\n", __func__);
    gpiod_set_value_cansleep(par->gpio.reset, 1);
    msleep(10);
    gpiod_set_value_cansleep(par->gpio.reset, 0);
    msleep(200);
    gpiod_set_value_cansleep(par->gpio.reset, 1);
    msleep(10);
}
```
### if the screen have some pxiel shift
```
static void fbtft_set_addr_win(struct fbtft_par *par, int xs, int ys, int xe,
			       int ye)
{
	write_reg(par, MIPI_DCS_SET_COLUMN_ADDRESS,
		  (xs >> 8) & 0xFF, xs & 0xFF, (xe >> 8) & 0xFF, xe & 0xFF);

	write_reg(par, MIPI_DCS_SET_PAGE_ADDRESS,
		  (ys+34 >> 8) & 0xFF, ys+34 & 0xFF, (ye+34 >> 8) & 0xFF, ye+34 & 0xFF);

	write_reg(par, MIPI_DCS_WRITE_MEMORY_START);
}
```


## device_tree orangepi-build/kernel/orange-pi-5.13-sunxi64/arch/arm64/boot/dts/allwinner/
### sun50i-h616-orangepi-zero2.dts  
```

&spi1 {
	#address-cells = <1>;
	#size-cells = <0>;
	status = "okay";

	spidev@1 {
		compatible = "spidev";
		status = "disabled";
		reg = <0>;
		spi-max-frequency = <1000000>;
	};

	st7789v: st7789v@0{
		compatible = "sitronix,st7789v";//这个要跟fb_st7789v.c对应上
		reg = <0>;
		status = "okay";
		spi-max-frequency = <48000000>;//最大速率
		spi-cpol;//spi模式
		spi-cpha;
		rotate = <270>;//屏幕翻转
		fps = <60>;//帧率
		buswidth = <8>;//数据位宽
		rgb;
		dc = <&pio 2 6 GPIO_ACTIVE_HIGH>;  // PA8 h3 gpio 0对应A、1对应B...
		reset = <&pio 2 5 GPIO_ACTIVE_HIGH>; // PC5
		cs = <&pio 2 8 GPIO_ACTIVE_LOW>;  // PC8 需要与下面的cs对应
		debug = <0>;
	};
};
```
### sun50i-h616.dtsi  
```
			spi1_pins: spi1-pins {
				pins = "PH6", "PH7", "PH8";
				function = "spi1";
				bias-pull-up;
			};

			spi1_cs_pin: spi1-cs-pin {
				pins = "PC8";
				function = "spi1";
			};
```


## menuconfig and build iso  
```
Device Drivers  --->　　
　　　　[*] Staging drivers  --->　　
　　　　　　　　<*>   Support for small TFT LCD display modules  --->
　　　　　　　　　　　　　　<*>   FB driver for the ST7789V LCD Controller 
```
```
Device Drivers  --->　　
　　　　[*] SPI support  --->　　
　　　　　　　　<*>   Allwinner SPI A31 controller
```

## Enter Ubuntu system and modify 
```

ls /dev/fb* 
//should have device like fb0

sudo orangepi-config  --->　
                    System  --->
                        Desktop Disable
```

```
sudo vim /usr/share/X11/xorg.conf.d/99-fbdev.conf

write down text below

Section "Device"  
  Identifier "myfb"
  Driver "fbdev"
  Option "fbdev" "/dev/fb0"
EndSection
```

```
sudo startx

switch to xfce Desktop
```


