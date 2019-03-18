#include <fcgi_stdio.h>
int main( int argc, char *argv[] )
{
   while( FCGI_Accept() >= 0 ) {
      printf( "Content-Type: text/plain\n\n" );
      printf( "Hello world in C\n" );
   }
   return 0;
}


/*
sudo apt install libfcgi-dev

gcc -o hello hello.c -lfcgi

*/
