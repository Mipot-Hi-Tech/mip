<!-- For .md file development refers to https://docs.github.com/en -->
# [MIPOT](https://www.mipot.com) C library for Single and Dual Core [MIP](https://mipot.com/en/products/?cat=110) Series modules.

The MIP Series is a family transceivers operating in the 868 MHz SRD Band (EU Version) or 915 MHz (US Version), optimized for very long range and low power applications.

The Series B/C/D are based on LoRa® RF Technology, providing ultra-long range spread spectrum communication and high interference immunity. 

The A Serie is based on the new wM-BUS OMS 2 protocol stack. The module supports various operating modes (S, T, R, C) to meet the requirements of one-way and two-way data communication, in stationary and mobile systems.

# Compatibility

### Single Core
 - [32001505AEU](https://mipot.com/en/products/mip-series/single-core/32001505aeu/) - 868 MHz Wireless M-Bus
 - [32001505BEU](https://mipot.com/en/products/mip-series/single-core/32001505beu/) - 868 MHz LoRaWAN
 - [32001505CEU](https://mipot.com/en/products/mip-series/single-core/32001505ceu/) - 868 MHz LoRa Mipot
 - [32001505DEU](https://mipot.com/en/products/mip-series/single-core/32001505deu/) - 868 MHz LoRa Modem
 - [32001505BUS](https://mipot.com/en/products/mip-series/single-core/32001505bus/) - 915 MHz LoRaWAN
 - [32001505CUS](https://mipot.com/en/products/mip-series/single-core/32001505cus/) - 915 MHz LoRa Mipot
 
 ### Dual Core
 - [32001506AEU](https://mipot.com/en/products/mip-series/dual-core/32001506aeu/) - 868 MHz Wireless M-Bus
 - [32001506BEU](https://mipot.com/en/products/mip-series/dual-core/32001506beu/) - 868 MHz LoRaWAN
 - [32001506CEU](https://mipot.com/en/products/mip-series/dual-core/32001506ceu/) - 868 MHz LoRa Mipot
 - [32001506DEU](https://mipot.com/en/products/mip-series/dual-core/32001506deu/) - 868 MHz LoRa Modem
 - [32001506BUS](https://mipot.com/en/products/mip-series/dual-core/32001506bus/) - 915 MHz LoRaWAN
 
# Library Structure for LoRa based modules

![img0](https://github.com/Mipot-Hi-Tech/mip/blob/master/img/img004.png)


# Library Structure for Wireless M-Bus based module

![img1](https://github.com/Mipot-Hi-Tech/mip/blob/master/img/img005.png)


# Modes of Operation

Modify the portable.c and portable.h files, based on your hardware.

Customize your application in the app folder.


# License
 
BSD-3-Clause


