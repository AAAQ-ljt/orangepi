# **GPS模块实验**

## **实验目的**

1、使用 SIM8262E-M2 5G 模组内置的 GNSS 功能，通过 AT 指令启动 GPS，在终端每秒打印经纬度、卫星数、速度等定位信息。

2、掌握 NMEA 0183 报文的串口流式解析方法：strtok 按 \n 拆包、strstr 匹配报文类型、parse_nmea_field 按下标抽取逗号分隔字段。

## **实验内容**

1、硬件接线：SIM8262E-M2 5G 模组 AT 通道接 /dev/ttyUSB2，GNSS 数据通道接 /dev/ttyUSB1，波特率统一 115200。

2、串口初始化：基于 termios 实现 8N1 无校验、无硬件流控、原始模式（raw mode）的串口配置函数 gps_init()。

3、GNSS 使能：通过 AT 通道发送 AT+CGPS=1 和 AT+CGPSOUT=64 两条指令，启动 SIM8262E-M2 内置 GNSS 并配置 NMEA 输出内容。

4、NMEA 报文规则学习：对照 gps讲解.docx，逐条理解 $ 开头、逗号分隔、* 校验、\r\n 结尾的 5 条硬性规则。

5、三类报文解析：分别实现 parse_gpgga（时间/经纬度/定位状态/卫星数/HDOP/海拔）、parse_gprmc（速度/航向/日期）、parse_gpgsv（总可见卫星数）三个解析函数。

6、主循环分发：在 gps_process 中用 read 批量读取缓冲区，strtok 按行拆分，再用 strstr 判断报文类型并分发到对应解析函数。

7、结果打印与验证：每秒调用 gps_print_info 输出一条格式化 GPS 信息，对比卫星实际分布与定位精度。

## **实验仪****器**

1、OrangePi 5 嵌入式开发板一台。

2、SIM8262E-M2 5G 通信模组一块（高通骁龙 X62 芯片，3GPP R16，集成多星座 GNSS），配套 M.2 转 USB 扩展板。

3、GNSS 有源天线一根（接 SIM8262E-M2 的 ANT3 接口，注意有字一面朝下）。

4、电脑一台，使用 MobaXterm 或其他 SSH 工具远程连接香橙派进行开发。

5、arm-linux-gnueabihf-gcc / aarch64-linux-gnu-gcc 交叉编译工具链，或香橙派本机 gcc 编译环境。

6、室外开阔场地或窗边（用于接收卫星信号，室内通常无法定位）。

## **实验原理**

SIM8262E-M2 启动 GNSS 后，通过数据串口持续输出 NMEA 0183 报文。每条报文通用格式如下：

$报文标识,字段1,字段2,字段3,...,最后字段*校验码\r\n

**代码硬性依赖的 5 条规则：**

1、每条报文以 $ 开头，\n 换行分隔，strtok(buf, "\n") 拆分单条数据包。

2、字段用英文逗号 , 分割，字段下标从 0 开始。

3、解析只读取 $ ~ * 校验符之间内容，parse_nmea_field 遇到 * 停止遍历。

4、串口流式输出，多包混杂在缓冲区，gps_process 一次性读取后分行解析。

5、常见报文类型：$GPGGA（定位信息）、$GPRMC（推荐最小定位）、$GPGSV（可见卫星）。

**Eg. 一条 $GPGGA 报文示例：**

$GPGGA,152230.000,3931.4160,N,11642.0720,E,1,08,1.02,22.0,M,,M,,*78\r\n

含义：UTC 15:22:30，纬度 39°31.4160′N，经度 116°42.0720′E，已定位，8 颗卫星，HDOP 1.02，海拔 22.0m。程序通过 parse_nmea_field 按逗号下标抽取各字段存入 GPS_Data 结构体，最后由 gps_print_info 格式化打印到终端。

## **实验步骤**

**一：工程宏定义、头文件与结构体**

gps.c 顶部 include 与宏定义（提取自 gps 素材图1、图5）：

**二：串口初始化 gps_init() — 8N1 原始模式**

该函数被 gps_enable_at（AT 通道）和 main（GPS 通道）各调用一次，打开指定设备节点并配置 termios 参数。

核心参数（termios 字段映射）：

| **下标**                    | **字段内容**                 | **代码操作**                  | **存入 GPS_Data**     |
| --------------------------- | ---------------------------- | ----------------------------- | --------------------- |
| **c_cflag**                 | CREAD \| CLOCAL              | opt.c_cflag = CREAD \| CLOCAL | 允许接收 + 忽略控制线 |
| **c_cflag**                 | CS8                          | opt.c_cflag \|= CS8           | 8 数据位              |
| **c_cflag**                 | ~PARENB                      | opt.c_cflag &= ~PARENB        | 无校验                |
| **c_cflag**                 | ~CSTOPB                      | opt.c_cflag &= ~CSTOPB        | 1 停止位              |
| **c_iflag**                 | ~(IXON\|IXOFF\|IXANY\|ICRNL) | opt.c_iflag &= ~(...)         | 无流控 + 不转 CR      |
| **c_lflag**                 | ~(ICANON\|ECHO\|ECHOE\|ISIG) | opt.c_lflag &= ~(...)         | 原始模式 + 不回显     |
| **c_oflag**                 | ~OPOST                       | opt.c_oflag &= ~OPOST         | 无输出后处理          |
| **cfsetispeed/cfsetospeed** | baudrate                     | cfsetispeed(&opt, baudrate)   | 输入输出波特率一致    |
| **tcflush**                 | TCIFLUSH                     | tcflush(fd, TCIFLUSH)         | 清空输入缓冲区        |

对应代码片段（提取自素材图6）：

**三：AT 指令通道使能 GPS — gps_enable_at()**

SIM8262E-M2 的 GNSS 默认不输出 NMEA 数据，需要通过 AT 通道发送两条指令启动。本函数先打开 AT 通道发完就关闭。

完整 AT 指令序列：

| **下标** | **字段内容**      | **代码操作**               | **存入 GPS_Data**                            |
| -------- | ----------------- | -------------------------- | -------------------------------------------- |
| **1**    | AT+CGPS=1\r\n     | cmd1 = "AT+CGPS=1\r\n"     | 开启 GPS 功能（冷启动）                      |
| **2**    | AT+CGPSOUT=64\r\n | cmd2 = "AT+CGPSOUT=64\r\n" | 配置 NMEA 输出内容（64 为 RMC+GGA+GSV 组合） |

对应代码片段（提取自素材图2）：

**四：通用字段抽取 — parse_nmea_field()**

该函数是三个解析函数的公用基础：在逗号分隔字符串中找到第 target_field 个字段，复制到 result，遇到 * 结束（避免把校验符拷进数据）。

对应代码片段（提取自素材图5 函数声明，结合素材图3/4 使用方式还原）：

**五：$GPGGA 数据包 → parse_gpgga()**

**数据包完整结构：**

$GPGGA,utc,lat,lat_dir,lon,lon_dir,fix_status,sat_cnt,hdop,alt,,,,*XX\r\n

**Eg.示例：**

$GPGGA,152230.000,3931.4160,N,11642.0720,E,1,08,1.02,22.0,M,,M,,*78\r\n

**字段下标与代码读取映射：**

| **下标** | **字段内容**    | **代码操作**                 | **存入 GPS_Data** |
| -------- | --------------- | ---------------------------- | ----------------- |
| **0**    | $GPGGA          | strstr 判断报文类型          | 不存储            |
| **1**    | UTC 时分秒.毫秒 | parse_nmea_field(line,1,...) | utc_time          |
| **2**    | 纬度 ddmm.mmmm  | parse_nmea_field(line,2,...) | latitude          |
| **3**    | 纬度方向 N/S    | parse_nmea_field(line,3,...) | lat_dir           |
| **4**    | 经度 dddmm.mmmm | parse_nmea_field(line,4,...) | longitude         |
| **5**    | 经度方向 E/W    | parse_nmea_field(line,5,...) | lon_dir           |
| **6**    | 定位状态 0/A/V  | 读入临时 char[2] 赋值 [0]    | status            |
| **7**    | 当前定位卫星数  | parse_nmea_field(line,7,...) | satellites        |
| **8**    | 水平精度 HDOP   | parse_nmea_field(line,8,...) | hdop              |
| **9**    | 海拔高度（米）  | parse_nmea_field(line,9,...) | altitude          |

对应代码片段（提取自素材图4，补充素材图3 前半段）：

**六：$GPRMC 数据包 → parse_gprmc()**

**数据包完整结构：**

$GPRMC,utc,fix,lat,lat_dir,lon,lon_dir,speed,course,date,,,*XX\r\n

**Eg.示例：**

$GPRMC,152230.000,A,3931.4160,N,11642.0720,E,0.00,0.00,030726,,,*62\r\n

**代码只提取 3 个下标字段：**

| **下标** | **字段内容**            | **代码操作**                 | **存入 GPS_Data** |
| -------- | ----------------------- | ---------------------------- | ----------------- |
| **7**    | 对地速度（节/节每小时） | parse_nmea_field(line,7,...) | speed             |
| **8**    | 航向角度 0~360°         | parse_nmea_field(line,8,...) | course            |
| **9**    | 日期 ddmmyy             | parse_nmea_field(line,9,...) | date              |

对应代码片段（提取自素材图4）：

**七：$GPGSV 数据包 → parse_gpgsv()**

**数据包完整结构：**

$GPGSV,总分片,当前分片,总可见卫星,卫星1,仰角,方位,信噪比,...*XX\r\n

**Eg.示例：**

$GPGSV,3,1,12,01,45,132,38,03,22,089,32,05,51,210,41,08,12,045,29*71\r\n

**代码仅读取下标 3：**

| **下标** | **字段内容**         | **代码操作**             | **存入 GPS_Data** |
| -------- | -------------------- | ------------------------ | ----------------- |
| **3**    | 接收机搜到的总卫星数 | 读取字符串后 atoi 转整数 | total_satellites  |

对应代码片段（提取自素材图4）：

**八：流式读取 + 报文分发 — gps_process()**

该函数是三个解析函数的唯一入口：每次 read 读满缓冲区，只要 len>0 就把 gps_module_detected 置为 1（静默检测策略），然后 strtok 按 \n 把缓冲区拆成独立报文，逐行 strstr 判断并分发。

分发流程映射：

| **下标** | **字段内容**    | **代码操作**                                              | **存入 GPS_Data**     |
| -------- | --------------- | --------------------------------------------------------- | --------------------- |
| **1**    | 清缓冲区 + read | memset(buf,0,sizeof(buf)); len=read(fd,buf,sizeof(buf)-1) | 留 1 字节给 \0        |
| **2**    | 模块检测        | if(len>0) gps_module_detected=1; if(len<=0) return;       | 无数据直接返回        |
| **3**    | 按行拆分        | char *line=strtok(buf,"\n");                              | 第一行                |
| **4a**   | GPGGA 分发      | if(strstr(line,"$GPGGA")) parse_gpgga(line,gps);          | 匹配则调用            |
| **4b**   | GPRMC 分发      | if(strstr(line,"$GPRMC")) parse_gprmc(line,gps);          | 匹配则调用            |
| **4c**   | GPGSV 分发      | if(strstr(line,"$GPGSV")) parse_gpgsv(line,gps);          | 匹配则调用            |
| **5**    | 取下一行        | line=strtok(NULL,"\n");                                   | while(line!=NULL)循环 |

对应代码片段（提取自素材图3）：

**九：格式化输出 — gps_print_info()**

工程化打印规则（提取自素材图7 注释）：

1、有模块时只打印 GPS 信息，绝不提示未检测到。

2、无模块时不打印任何东西（避免刷屏干扰其他线程输出）。

对应代码片段（提取自素材图7）：

**十：主函数入口 — main()**

main 函数整合所有步骤：先通过 AT 通道启动 GPS（sleep(1) 等冷启动），再打开 GPS 数据通道，创建一个 GPS_Data 实例，进入无限循环，每秒 process+print 一次。

对应代码片段（提取自素材图1）：

## **实验结果**

![img](file:///C:\Users\luo20\AppData\Local\Temp\ksohtml64068\wps2.jpg) 

## **实验结果分析**

1、AT 指令启动延迟分析：SIM8262E-M2 处理 AT+CGPS=1 需要切换射频链路并启动内部 GNSS 固件流程，两条 AT 指令之间必须加 usleep(300000) 等待模组处理完成。若不加延时，第二条 AT+CGPSOUT=64 可能在模组尚未就绪时写入而被丢弃，导致 GNSS 启动失败、数据通道无 NMEA 输出。300ms 是工程经验值，部分模组冷启动可能需要更长。

2、双串口架构的必要性：SIM8262E-M2 通过 USB 枚举出多个独立串口，AT 通道（/dev/ttyUSB2）用于发送控制指令，数据通道（/dev/ttyUSB1）专用于接收 NMEA 字节流。两者分离避免了 AT 指令响应与 NMEA 数据混杂在同一缓冲区导致的解析困难，gps_enable_at 发完指令即 close AT 通道，后续 gps_process 只专注数据通道读取。

3、strtok + strstr 流式解析的健壮性：串口缓冲区中多条 NMEA 报文混杂，strtok(buf, "\n") 按换行符拆分成独立报文，再用 strstr(line, "$GPGGA") 判断报文类型分发。只传 "\n" 而非 "\r\n" 是因为 strtok 分隔符参数中每个字符都当独立分隔符，\r 会留在行尾但不影响 strstr 的前向匹配，兼容 \r\n 和 \n 两种结尾的模组。

4、parse_nmea_field 边界安全设计：该函数按下标抽取逗号分隔字段，遇 * 停止遍历（不把校验码拷进数据），并用 max_len 限制写入长度防止缓冲区溢出。定位状态字段（下标6）只有一个字符 A/V，先读入临时 char[2] 再取 [0] 赋值给 char status，避免了直接写入单字符变量时 \0 越界覆盖相邻内存。

5、静默打印策略的工程意义：gps_module_detected 标志仅在 len>0 时置 1 且从不清零，gps_print_info 据此决定是否输出。有模块时只打印 GPS 信息绝不提示"未检测到"，无模块时完全静默。这是因为 GPS 线程与小车其他线程（IMU、视觉、控制）共享终端，持续打印"No GPS"会淹没关键调试信息。

6、定位精度与卫星数的关系：实验中 satellites（GPGGA 下标7）为当前用于定位的卫星数，total_satellites（GPGSV 下标3）为接收机搜到的总可见卫星数。两者比值越高（≥60%）说明信号质量越好、定位精度越高。室内或遮挡环境下 satellites 可能为 0 且 status=V（未定位），移至室外开阔处后通常 30 秒~2 分钟内完成冷启动定位。

 

 