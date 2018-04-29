#!/usr/bin/perl

use strict;

my $headers = {
    'C#=' => 'canMsg',
    'T#=' => 'timeMsg',
};
my $headerLen = 3;

my $inBetween = '';
my $startup = 0;
binmode( STDIN );

$/ = \1;
while( my $header = readHeader() ) {
    if( $headers->{$header} eq 'timeMsg' ) {
        my $millis = readUnsignedLong();
        my $unixtime = readUnsignedLong();
        $startup = $unixtime - int( $millis / 1000 );
        print "Millis $millis == Epoch $unixtime\n";
        print "File opened at " . millisToTimeString( $millis ) . "\n";
        print "Startup at " . millisToTimeString( 0 ) . "\n";
    }
    elsif( $headers->{$header} eq 'canMsg' ) {
        my $log = '';
        my $millis = readUnsignedLong();
        my $canNum = readUnsignedChar();
        $log .= millisToTimeString( $millis );
        $log .= ":[$canNum]";
        my $messageId = readUnsignedLong();
        $log .= ": " . sprintf( '0x%03X', $messageId );
        my $dataLen = readUnsignedChar();
        $log .= "[$dataLen]: ";
        for( my $i = 0; $i < $dataLen; $i++ ) {
            my $data = readUnsignedChar();
            $log .= sprintf( '%02X', $data ) . ' ';
        }
        my $newline = readUnsignedChar();
        if( $newline != 10 ) {
            print "Error in file, newline not at end of data\n";
        }
        $log .= "\n";
        print $log;
    }

}

sub readUnsignedLong {
    read( STDIN, my $bytes, 4 );
    my $long = unpack( 'V', $bytes );
    return $long;
}

sub readUnsignedChar {
    read( STDIN, my $bytes, 1 );
    return unpack( 'C', $bytes );
}

sub millisToTimeString {
    my $millis = shift;
    my $seconds = int( $millis / 1000 );
    $millis = $millis % 1000;
    my ($sec,$min,$hour,$mday,$mon,$year,$wday,$yday,$isdst) =
                                                gmtime($startup + $seconds);
    return sprintf( '%02d:%02d:%02d.%03d', $hour, $min, $sec, $millis );
}

sub addMisc {
    my $misc = shift;
    $inBetween .= $misc;
    if( substr( $inBetween, -1 ) eq "\n" ) {
        print $inBetween;
        $inBetween = '';
    }
}
    

sub readHeader {
    my $header = '';
    my $bytesRead = 0;
    do {
        $bytesRead = read( STDIN, my $char, 1 );
        $header .= $char;
        if( length( $header ) > $headerLen ) {
            addMisc( substr( $header, 0, length( $header ) - $headerLen ) );
            $header = substr( $header, $headerLen * -1 );
        }
        #print "Cur header: '$header' (BytesRead: $bytesRead)\n";
        if( defined( $headers->{$header} ) ) {
            return $header;
        }
    } while( $bytesRead > 0 );
}

