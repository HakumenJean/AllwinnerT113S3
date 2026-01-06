/*
 * Copyright (c) 2023, RT-Thread Development Team
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Change Logs:
 * Date           Author       Notes
 * 2026-01-06     HakumenJean  first version
 */

#include <rtthread.h>
#include <rthw.h>
#include <rtdevice.h>

#ifdef RT_USING_LWIP

#include <string.h>
#include <lwip/sockets.h>
#include "lwip/init.h"
#include "lwip/netif.h"

#define LWIP_DEMO_RX_BUFSIZE         200                        /* 最大接收数据长度 */
uint8_t g_lwip_demo_recvbuf[LWIP_DEMO_RX_BUFSIZE];

#define LWIP_DEMO_PORT               502                       /* 连接的本地端口号 */

int g_sock_conn = -1;                          /* 请求的 socked */

#undef FD_SETSIZE
#define FD_SETSIZE 5

void lwip_select_server()
{
    int sock_fd, client_fd, maxfd;
    int length, i;
    struct sockaddr_in server_addr;
    struct sockaddr_in client_addr;

    socklen_t client_addr_len;
    fd_set all_set, read_set;

    client_addr_len = sizeof(client_addr);

    int clientfds[FD_SETSIZE - 1];      //FD_SETSIZE里面包含了服务器的fd

    int so_keepalive_val = 1;   //使能心跳机制
    int tcp_keepalive_idle = 1; //发送心跳空闲周期 S
    int tcp_keepalive_intvl = 1;//发送心跳间隔 S
    int tcp_keepalive_cnt = 1;  //重发次数
    int tcp_nodelay = 1;        //不延时发送到合并包

    sock_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); /* 建立一个新的socket连接 */

    int reuseport = 1;
    setsockopt(sock_fd, SOL_SOCKET, SO_REUSEADDR, (const char *)&reuseport, sizeof(int));

    rt_kprintf("socket is %d \r\n", sock_fd);

    rt_memset(&server_addr, 0, sizeof(server_addr));        /* 将服务器地址清空 */
    server_addr.sin_family = AF_INET;                    /* 地址家族 */
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);     /* 注意转化为网络字节序 */
    server_addr.sin_port = htons(LWIP_DEMO_PORT);        /* 使用SERVER_PORT指定为程序头设定的端口号 */

    bind(sock_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)); /* 建立绑定 */

    listen(sock_fd, FD_SETSIZE - 1);   /* 监听连接请求 */

    maxfd = sock_fd;            //初始化 maxfd 等于 sock_fd

    FD_ZERO(&all_set);          //清空fdset
    FD_SET(sock_fd, &all_set);      //把sfd文件描述符添加到集合中

    for(i = 0; i < FD_SETSIZE - 1; i++){    //初始化客户端fd的集合
        clientfds[i] = -1;  //初始化为-1
    }

    //每次select返回之后，fd_set集合就会变化，再select时，就不能使用，
    //所以我们要保存设置fd_set 和 读取的fd_set
    while(1)
    {
        read_set = all_set;
        select(maxfd + 1, &read_set, NULL, NULL, NULL);

        if(FD_ISSET(sock_fd, &read_set))    //判断监听的套接字是否有数据
        {
            client_fd = accept(sock_fd, (struct sockaddr *)&client_addr, &client_addr_len); //有客户端进行连接了
            if(client_fd < 0){
                rt_kprintf("accept socket error\r\n");
                continue;   //继续select
            }

            for(i = 0; i < FD_SETSIZE - 1; i++)     //把新的cfd 保存到cfds集合中
            {
                if(clientfds[i] == -1)
                {
                    rt_kprintf("new client connect fd = %d\r\n", client_fd);
                    clientfds[i] = client_fd;
                    FD_SET(client_fd, &all_set);            //把新的cfd 添加到fd_set集合中

                    //使能心跳机制
                    setsockopt(clientfds[i], SOL_SOCKET, SO_KEEPALIVE, &so_keepalive_val, sizeof(int));

                    //配置心跳检测参数   默认参数时间很长
                    setsockopt(clientfds[i], IPPROTO_TCP, TCP_KEEPIDLE, &tcp_keepalive_idle, sizeof(int));
                    setsockopt(clientfds[i], IPPROTO_TCP, TCP_KEEPINTVL, &tcp_keepalive_intvl, sizeof(int));
                    setsockopt(clientfds[i], IPPROTO_TCP, TCP_KEEPCNT, &tcp_keepalive_cnt, sizeof(int));
                    setsockopt(clientfds[i], IPPROTO_TCP, TCP_NODELAY, &tcp_nodelay, sizeof(int));

                    maxfd = (client_fd > maxfd) ? client_fd : maxfd;        //更新要select的maxfd

                    break;  //退出，不需要添加
                }
            }
            if(i == FD_SETSIZE - 1)
            {
                rt_kprintf("new client connet but no memory to save\r\n");
                closesocket(client_fd);
            }
        }

        //遍历所有的客户端文件描述符
        for(i = 0; i < FD_SETSIZE - 1; i++)
        {
            if(clientfds[i] != -1 && FD_ISSET(clientfds[i], &read_set))
            {
                length = recv(clientfds[i], (unsigned int *)g_lwip_demo_recvbuf, sizeof(g_lwip_demo_recvbuf), 0);   /* 将收到的数据放到接收Buff */
                //rt_kprintf("socket %d rcv data size %d\r\n", clientfds[i], length);
                if(length <= 0)
                {
                    rt_kprintf("recv error, client %d is closed\r\n", clientfds[i]);
                    FD_CLR(clientfds[i], &all_set);
                    closesocket(clientfds[i]);
                    clientfds[i] = -1;
                }
                else
                {
                    length = send(clientfds[i], g_lwip_demo_recvbuf, length, 0); /* 发送数据 */
                    if(length < 0)
                    {
                        rt_kprintf("send error, client %d is closed\r\n", clientfds[i]);
                        FD_CLR(clientfds[i], &all_set);
                        closesocket(clientfds[i]);
                        clientfds[i] = -1;
                    }
                }
            }
        }
    }
}

void lwip_demo()
{
    struct sockaddr_in server_addr; /* 服务器地址 */
    struct sockaddr_in conn_addr;   /* 连接地址 */
    socklen_t addr_len;             /* 地址长度 */
    int err;
    int length;
    int sock_fd;

    int so_keepalive_val = 1;   //使能心跳机制
    int tcp_keepalive_idle = 3; //发送心跳空闲周期 S
    int tcp_keepalive_intvl = 3;//发送心跳间隔 S
    int tcp_keepalive_cnt = 3;  //重发次数
    int tcp_nodelay = 1;        //不延时发送到合并包

    sock_fd = socket(AF_INET, SOCK_STREAM, IPPROTO_TCP); /* 建立一个新的socket连接 */

    //使能心跳机制
    setsockopt(sock_fd, SOL_SOCKET, SO_KEEPALIVE, &so_keepalive_val, sizeof(int));

    //配置心跳检测参数   默认参数时间很长
    setsockopt(sock_fd, IPPROTO_TCP, TCP_KEEPIDLE, &tcp_keepalive_idle, sizeof(int));
    setsockopt(sock_fd, IPPROTO_TCP, TCP_KEEPINTVL, &tcp_keepalive_intvl, sizeof(int));
    setsockopt(sock_fd, IPPROTO_TCP, TCP_KEEPCNT, &tcp_keepalive_cnt, sizeof(int));
    setsockopt(sock_fd, IPPROTO_TCP, TCP_NODELAY, &tcp_nodelay, sizeof(int));

    rt_kprintf("socket is %d \r\n", sock_fd);

    rt_memset(&server_addr, 0, sizeof(server_addr));        /* 将服务器地址清空 */
    server_addr.sin_family = AF_INET;                    /* 地址家族 */
    server_addr.sin_addr.s_addr = htonl(INADDR_ANY);     /* 注意转化为网络字节序 */
    server_addr.sin_port = htons(LWIP_DEMO_PORT);        /* 使用SERVER_PORT指定为程序头设定的端口号 */

    err = bind(sock_fd, (struct sockaddr *)&server_addr, sizeof(server_addr)); /* 建立绑定 */

    if (err < 0)                /* 如果绑定失败则关闭套接字 */
    {
        rt_kprintf("bind err : %d \r\n", err);
        closesocket(sock_fd);   /* 关闭套接字 */
        return;
    }

    err = listen(sock_fd, 4);   /* 监听连接请求 */

    if (err < 0)                /* 如果监听失败则关闭套接字 */
    {
        rt_kprintf("listen err : %d \r\n", err);
        closesocket(sock_fd);   /* 关闭套接字 */
        return;
    }

    while(1)
    {
        addr_len = sizeof(struct sockaddr_in); /* 将链接地址赋值给addr_len */

        rt_kprintf("start accept \r\n");

        g_sock_conn = accept(sock_fd, (struct sockaddr *)&conn_addr, &addr_len); /* 对监听到的请求进行连接，状态赋值给g_sock_conn */

        rt_kprintf("accept ok, connect sock is %d\r\n", g_sock_conn);

        if (g_sock_conn < 0) /* 状态小于0代表连接故障，此时关闭套接字 */
        {
            closesocket(sock_fd);
            return;
        }

        while (1)
        {
            rt_memset(g_lwip_demo_recvbuf,0,LWIP_DEMO_RX_BUFSIZE);
            length = recv(g_sock_conn, (unsigned int *)g_lwip_demo_recvbuf, sizeof(g_lwip_demo_recvbuf), 0); /* 将收到的数据放到接收Buff */

            if (length <= 0)
            {
                goto atk_exit;
            }

            send(g_sock_conn, g_lwip_demo_recvbuf, length, 0); /* 发送数据 */
        }
atk_exit:
        rt_kprintf("sock client %d is disconnect\r\n", g_sock_conn);
        if (g_sock_conn >= 0)
        {
            closesocket(g_sock_conn);
            g_sock_conn = -1;
        }
    }
}

static void lwip_demo_thread_entry(void *parameter)
{
    //lwip_demo();            /* lwip测试代码 */
    lwip_select_server();       // 并发服务器

    while (1)
    {
        rt_thread_mdelay(5);
    }
}

void lwip_demo_start()
{
    rt_thread_t tid = rt_thread_create("lwip demo", lwip_demo_thread_entry, RT_NULL, 8192, 16, 5);
    if(tid != RT_NULL) {
        rt_thread_startup(tid);
        rt_kprintf("lwip demo start\r\n");
    }
}

#endif


