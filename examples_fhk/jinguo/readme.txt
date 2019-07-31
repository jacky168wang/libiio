Complier:
    make clean
    make
Target:
    phy_init.arm
    phy_stream.arm
Common useage in board:
    ./phy_init.arm -p ptx12288.txt
    ./phy_stream.arm -v 3
other declare:
    ./phy_init.arm -p ptx12288.txt (init ad9371 profile with input clock 122.88M)
    ./phy_stream.arm -v 3 (open the ad9371 rx tx and orx stream)
 
