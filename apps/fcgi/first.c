#include <stdlib.h>

#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif

extern char **environ;

#include <fcgi_stdio.h>


int main( int argc, char *argv[] )
{
	int count = 0;
	
	char **env = environ;
	
	while( FCGI_Accept() >= 0 )
	{
#if 0
		printf( "Content-Type: text/plain\n\n" );
		printf( "Hello world in C with Fast CGI\n" );
#else      
		printf( "Content-Type: application/json\n\n" );
		printf( "{\"system\":\"Mux500774\"," );
		for ( ; *env != NULL; env++)
		{
			char *value = strchr(*env, '=');
			if(value)
			{
				printf("\"%.*s\":\"%s\",",  (value-*env), *env,  (value+1) );
//				printf("\%s\":\"%s\",",  *env,  (value+1) );
			}
//			printf("\"%s\",",  *env );
		}
		
		printf( "\"FastCgiCount\":%d}", count );
#endif      
		count++;
	}
	
	return 0;
}


/*
sudo apt install libfcgi-dev
gcc -o hello hello.c -lfcgi
*/

