#!/usr/bin/perl

use strict;

my $file = shift;
print STDERR "Using file $file\n";
my $headers = {
    'C#=' => 'canMsg',
    'T#=' => 'timeMsg',
};
my $headerLen = 3;

my $startup = 0;

$/ = \1;
open( my $fh, '<:raw', "$file" ) or die( "error opening file $!" );
binmode( $fh );
while( my $header = readHeader( $fh ) ) {
    if( $headers->{$header} eq 'timeMsg' ) {
        my $millis = readUnsignedLong( $fh );
        my $unixtime = readUnsignedLong( $fh );
        $startup = $unixtime - int( $millis / 1000 );
        print "Millis $millis == Epoch $unixtime\n";
        print "File opened at " . millisToTimeString( $millis ) . "\n";
        print "Startup at " . millisToTimeString( 0 ) . "\n";
    }
    elsif( $headers->{$header} eq 'canMsg' ) {
        my $log = '';
        my $millis = readUnsignedLong( $fh );
        my $canNum = readUnsignedChar( $fh );
        $log .= millisToTimeString( $millis );
        $log .= ":[$canNum]";
        my $messageId = readUnsignedLong( $fh );
        $log .= ": " . sprintf( '0x%X', $messageId );
        my $dataLen = readUnsignedChar( $fh );
        $log .= "[$dataLen]: ";
        for( my $i = 0; $i < $dataLen; $i++ ) {
            my $data = readUnsignedChar( $fh );
            $log .= sprintf( '%X', $data ) . ' ';
        }
        my $newline = readUnsignedChar( $fh );
        if( $newline != 10 ) {
            print "Error in file, newline not at end of data\n";
        }
        $log .= "\n";
        print $log;
    }

}

sub readUnsignedLong {
    my $fh = shift;
    read( $fh, my $bytes, 4 );
    my $long = unpack( 'V', $bytes );
    return $long;
}

sub readUnsignedChar {
    my $fh = shift;
    read( $fh, my $bytes, 1 );
    return unpack( 'C', $bytes );
}

sub millisToTimeString {
    my $millis = shift;
    my $seconds = int( $millis / 1000 );
    $millis = $millis % 1000;
    my ($sec,$min,$hour,$mday,$mon,$year,$wday,$yday,$isdst) =
                                                gmtime($startup + $seconds);
    return sprintf( '%02d:%02d:%02d.%d', $hour, $min, $sec, $millis );
}

sub readHeader {
    my $fh = shift;
    my $header = '';
    my $bytesRead = 0;
    do {
        $bytesRead = read( $fh, my $char, 1 );
        $header .= $char;
        if( length( $header ) > $headerLen ) {
            $header = substr( $header, $headerLen * -1 );
        }
        #print "Cur header: '$header' (BytesRead: $bytesRead)\n";
        if( defined( $headers->{$header} ) ) {
            return $header;
        }
    } while( $bytesRead > 0 );
}

