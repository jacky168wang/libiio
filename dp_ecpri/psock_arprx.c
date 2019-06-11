/*
 * raw_socket application example for arp_request RX
 *
 * Copyright (C) 2018~2020 FACC Inc.
 * Author: Jacky Wang <kenwj@sina.com>
 *
 * License: GPL, version 2.1
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/if_ether.h>
#include <net/if_arp.h>
#include <net/ethernet.h>

/* 以太网?首部?度 */
#define ETHER_HEADER_LEN sizeof(struct ethhdr)
/* 整?arp?构?度 */
#define ETHER_ARP_LEN sizeof(struct ether_arp)
/* 以太网 + 整?arp?构?度 */
#define ETHER_ARP_PACKET_LEN ETHER_HEADER_LEN + ETHER_ARP_LEN
/* IP地址?度 */
#define IP_ADDR_LEN 4

void err_exit(const char *err_msg)
{
    perror(err_msg);
    exit(1);
}

int main(void)
{
    struct ether_arp *arp_packet;
    char buf[ETHER_ARP_PACKET_LEN];
    int sock_raw_fd, ret_len, i;

    if ((sock_raw_fd = socket(PF_PACKET, SOCK_RAW, htons(ETH_P_ARP))) == -1)
    {
        err_exit("socket()");
		printf("ERROR!!");
    }

    while (1)
    {
        bzero(buf, ETHER_ARP_PACKET_LEN);
        ret_len = recv(sock_raw_fd, buf, ETHER_ARP_PACKET_LEN, 0);
        if (ret_len > 0)
        {
            /* ?去以太?部 */
            arp_packet = (struct ether_arp *)(buf + ETHER_HEADER_LEN);
            /* arp操作??2代表arp?答 */
            if (ntohs(arp_packet->arp_op) == 2)
            {
                printf("==========================arp replay======================\n");
                printf("from ip:");
                for (i = 0; i < IP_ADDR_LEN; i++)
                    printf(".%u", arp_packet->arp_spa[i]);
                printf("\r\nfrom mac");
                for (i = 0; i < ETH_ALEN; i++)
                    printf(":%02x", arp_packet->arp_sha[i]);
                printf("\n");
            }
        }
    }

    close(sock_raw_fd);
    return 0;
}
