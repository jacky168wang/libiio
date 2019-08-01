/*
IBM grants you a nonexclusive copyright license to use all programming code 
examples from which you can generate similar function tailored to your own 
specific needs.
SUBJECT TO ANY STATUTORY WARRANTIES WHICH CANNOT BE EXCLUDED, IBM, ITS PROGRAM
DEVELOPERS AND SUPPLIERS MAKE NO WARRANTIES OR CONDITIONS EITHER EXPRESS OR 
IMPLIED, INCLUDING BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OR CONDITIONS OF
MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE, AND NON-INFRINGEMENT, 
REGARDING THE PROGRAM OR TECHNICAL SUPPORT, IF ANY.  
UNDER NO CIRCUMSTANCES IS IBM, ITS PROGRAM DEVELOPERS OR SUPPLIERS LIABLE FOR 
ANY OF THE FOLLOWING, EVEN IF INFORMED OF THEIR POSSIBILITY:  
    LOSS OF, OR DAMAGE TO, DATA;
    DIRECT, SPECIAL, INCIDENTAL, OR INDIRECT DAMAGES, OR FOR ANY ECONOMIC 
    	CONSEQUENTIAL DAMAGES; OR
    LOST PROFITS, BUSINESS, REVENUE, GOODWILL, OR ANTICIPATED SAVINGS.
SOME JURISDICTIONS DO NOT ALLOW THE EXCLUSION OR LIMITATION OF DIRECT, 
INCIDENTAL, OR CONSEQUENTIAL DAMAGES, SO SOME OR ALL OF THE ABOVE LIMITATIONS 
OR EXCLUSIONS MAY NOT APPLY TO YOU. 
*/

/**************************************************************************/
/* Worker job that uses the recvmsg to process client requests            */
/**************************************************************************/
#include <stdio.h>
#include <stdlib.h>
#include <sys/socket.h>

main (int argc, char *argv[])
{
   int    rc, len;
   int    worker_sd, pass_sd;
   char   buffer[80];
   struct iovec   iov[1];
   struct msghdr  msg;

   /*************************************************/
   /* One of the socket descriptors that was        */
   /* returned by socketpair(), is passed to this   */
   /* worker job as descriptor 0.                   */
   /*************************************************/
   worker_sd = 0;

   /*************************************************/
   /* Initialize message header structure           */
   /*************************************************/
   memset(&msg,   0, sizeof(msg));
   memset(iov,    0, sizeof(iov));

   /*************************************************/
   /* The recvmsg() call will NOT block unless a    */
   /* non-zero length data buffer is specified      */
   /*************************************************/
   iov[0].iov_base = buffer;
   iov[0].iov_len  = sizeof(buffer);
   msg.msg_iov     = iov;
   msg.msg_iovlen  = 1;

   /*************************************************/
   /* Fill in the msg_accrights fields so that we   */
   /* can receive the descriptor                    */
   /*************************************************/
   msg.msg_accrights    = (char *)&pass_sd;
   msg.msg_accrightslen = sizeof(pass_sd);

   /*************************************************/
   /* Wait for the descriptor to arrive             */
   /*************************************************/
   printf("Waiting on recvmsg\r\n");
   rc = recvmsg(worker_sd, &msg, 0);
   if (rc < 0)
   {
      perror("recvmsg() failed");
      close(worker_sd);
      exit(-1);
   }
   else if (msg.msg_accrightslen <= 0)
   {
      printf("Descriptor was not received\r\n");
      close(worker_sd);
      exit(-1);
   }
   else
   {
      printf("Received descriptor = %d\r\n", pass_sd);
   }

   /*************************************************/
   /* Receive a message from the client             */
   /*************************************************/
   printf("Wait for client to send us a message\r\n");
   rc = recv(pass_sd, buffer, sizeof(buffer), 0);
   if (rc <= 0)
   {
      perror("recv() failed");
      close(worker_sd);
      close(pass_sd);
      exit(-1);
   }
   printf("<%s>\n", buffer);

   /*************************************************/
   /* Echo the data back to the client              */
   /*************************************************/
   printf("Echo it back\r\n");
   len = rc;
   rc = send(pass_sd, buffer, len, 0);
   if (rc <= 0)
   {
      perror("send() failed");
      close(worker_sd);
      close(pass_sd);
      exit(-1);
   }

   /*************************************************/
   /* Close down the descriptors                    */
   /*************************************************/
   close(worker_sd);
   close(pass_sd);
}

