
#tcpdump -i lo -X -vv
tcpdump -i eth0 -X -vv

#See the list of interfaces on which tcpdump can listen:
tcpdump -D
tcpdump -i eth0 # Listen on interface eth0
tcpdump -i any #Listen on any available interface (cannot be done in promiscuous mode)

#Be verbose and print the data of each packet in both hex and ASCII
tcpdump -v -X # excluding the link level header:
tcpdump -v -XX # including the link level header
tcpdump -q #Be less verbose

tcpdump -c 100 # Limit the capture to 100 packets

tcpdump -w capture.cap # Record the packet capture to a file
tcpdump -v -w capture.cap # and display on-screen how many packets have been captured in real-time

tcpdump -r capture.cap # Display the packets of a file
tcpdump -vvv -r capture.cap #Display the packets using the maximum detail

tcpdump -n # Display IP addresses and port numbers
tcpdump -n dst host 192.168.1.1
tcpdump -n src host 192.168.1.1
tcpdump -n host 192.168.1.1
tcpdump -n dst net 192.168.1.0/24
tcpdump -n src net 192.168.1.0/24
tcpdump -n net 192.168.1.0/24
tcpdump -n dst port 23
tcpdump -n dst portrange 1-1023
tcpdump -n tcp dst portrange 1-1023
tcpdump -n udp dst portrange 1-1023
tcpdump -n "dst host 192.168.1.1 and dst port 23"
tcpdump -n "dst host 192.168.1.1 and (dst port 80 or dst port 443)"

tcpdump -v icmp
tcpdump -v arp
tcpdump -v "icmp or arp"

tcpdump -n "broadcast or multicast"

tcpdump -s 500 # Capture 500 bytes of data for each packet rather than the default of 68 bytes
tcpdump -s 0 # Capture all bytes of data within the packet