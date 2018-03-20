finalall: finalserver finalclient

finalserver: finalserver.c
	gcc finalserver.c -o finalserver

finalclient: finalclient.c
	gcc finalclient.c -o finalclient

finalclean:
	rm -f finalclient finalserver
