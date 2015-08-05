#include "messaging.h"
#include <stdio.h>

int main( int argc, char ** argv ) {

	libnxt_error error = init_messaging();
	if ( error ) {
		printf( "Error initialising: %s\n", libnxt_error_message( error ) );
		return 1;
	}

	unsigned char endPoints[2] = {0, 8};
	error = send( endPoints, sizeof ( endPoints ) );

	unsigned char * report = NULL;
	uint16_t reportLength = 0;

	size_t i;
	do {
		error = receive( &report, &reportLength );
		if ( error ) {
			printf( "Error receiving: %s\n", libnxt_error_message( error ) );
		} else {
			for ( i = 0; i < reportLength; i++ ) {
				printf( "%c", report[i] );
			}
			printf( "\n" );
			free_message( report );
			report = NULL;
		}
	} while( ( ! error ) && reportLength > 0 );

	exit_messaging();
	return ( error ? 1 : 0 );
}
