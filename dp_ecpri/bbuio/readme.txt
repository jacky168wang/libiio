This is an attempt to extract the BBU RRU IO relevant portion from CRAN code base.

The working copy of source code is:
bbuio.cpp
bbuio.h

Which has quite a lot of MACRO enable/disable option, which is defined in the attached file:
tb_config.h

To ease the understanding of the bbuio.cpp, the package provides also the bbuio_partial.cpp which has the major RRU packet handling thread and function call extracted, with most of the MACRO resolved.

It is basically a almost always busy thread that occupies one of the Intel processor core, and checks on the dpdk receive queue for:
TIME PACKET
PUSCH PACKET

and schedule time to do dl_transmission, which send via dpdk socket the 
PDSCH PACKET