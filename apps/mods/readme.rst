README for fastcgi and fcgid modules for Apache
###################################################

fastcgi module
------------------

Configuration
::

	<IfModule fastcgi_module>                                             
	  FastCgiIpcDir /var/www/apis                                           
	  <Directory /var/www/apis>                                                                 
		SetHandler fastcgi-script
		Options +ExecCGI                                                      
	  </Directory>                                                              
	  <Location "/apis">                                                                  
		Require all granted
		Options +ExecCGI 
	  </Location>                                                                 
	  FastCgiServer /var/www/apis/muxApis -flush
	  Options +ExecCGI 
	  AliasMatch "/apis(.*)" "/var/www/apis/muxApis"                      
	</IfModule> 

Everything is OK: create new process of fcgi- and muxApis; but cgi is not executed, only download;


fcgid module
------------------

Configuration

::

	<IfModule mod_fcgid.c>   
	  Scriptsock cgisock
	  <Directory /var/www/apis>
		SetHandler fcgid-script
		Options +ExecCGI
		Require all granted
	  </Directory>                                              
	  <Location "/apis">
		Require all granted
		SetHandler fcgid-script                                              
		Options +ExecCGI 
	  </Location>                   
	  ScriptAliasMatch "/apis(.*)" "/var/www/apis/muxApis"
	</IfModule>

	
Everything is OK: create new process of fcgi- and muxApis; but cgi is not executed, only download;
