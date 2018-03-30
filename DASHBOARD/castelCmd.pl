#!/usr/bin/perl 

# but final : 
#     + ihm avec slide (reglage pwm), envoi pwm ou non, et affichage des paramètres
#     + mainloop avec
#      ° un repeat qui envoie toutes les 100ms un message pwm
#      ° la FHD de la liaison serie attachée à une callback qui récupère les données et les affiche
#
#
#

use strict;
use warnings;
use feature ':5.22';
use Device::SerialPort;
use Modern::Perl '2015';
use Tk;
use Tk::ProgressBar;
use Carp qw/longmess cluck confess/;
use POSIX;

no warnings 'experimental::smartmatch';

sub fletcher16 ($$);
sub statusFunc ($$$$$$$);
sub initSerial ($);
sub serialCb();
sub castelLinkMessageCb($);
sub geneCastelLinkMsgsCB();
sub getSerial();
sub generateGui();
sub generatePanel ();
sub generateOneServoFrame ($$);
sub labelLabelFrame ($$$$;$) ;
sub fhbits(@);

my $mw;
my $mwf;

my %options;
my %tkObject = (
    "clink0" => 0,
    "clink1" => 0,
    "clinkOn0" => 1,
    "clinkOn1" => 1,
    );


my @varData = (
    {
	'bat_voltage' => 0,	
	'ripple_voltage' => 0,	
	'current' => 0,	
	'throttle' => 0,	
	'power' => 0,		
	'rpm' => 0,		
	'bec_voltage' => 0,	
	'bec_current' => 0,	
	'temperature' => 0,
    }, {
	'bat_voltage' => 0,	
	'ripple_voltage' => 0,	
	'current' => 0,	
	'throttle' => 0,	
	'power' => 0,		
	'rpm' => 0,		
	'bec_voltage' => 0,	
	'bec_current' => 0,	
	'temperature' => 0,
   }
);



my $serialName = $ARGV[0] // "/dev/ttyACMx";
#my $serialHandle;


$serialName = getSerial () if $serialName eq "/dev/ttyACMx";

say "DBG> serialName = $serialName";
#sysopen ($serialHandle,  $serialName, O_RDWR);
my $serial;

do {
    $serial = initSerial ($serialName);
    sleep (1) unless $serial;
} unless defined $serial;

my $selectReadBits = fhbits(*FHD);

generateGui();

my $dbgBuf;


#$mw->fileevent(\*FHD, 'readable', \&serialCb) ;
$mw->repeat (100, \&geneCastelLinkMsgsCB);

Tk::MainLoop;






#                 _____                  _    _______   _            
#                |  __ \                | |  |__   __| | |           
#                | |__) |   ___   _ __  | |     | |    | | _         
#                |  ___/   / _ \ | '__| | |     | |    | |/ /        
#                | |      |  __/ | |    | |     | |    |   <         
#                |_|       \___| |_|    |_|     |_|    |_|\_\        
sub generateGui()
{
    $mw = MainWindow->new;
    $mw->wm (title => "castel link live");
    my $w = $mw->screenwidth;
    my $h = $mw->screenheight;

    $mw->MoveToplevelWindow (0,0);

    $mwf =  $mw->Frame ()->pack(-side => 'left', -anchor => 'w');
    generatePanel ();
}


sub generatePanel ()
{
    my $outerFrame = $mwf->Frame ();
    $outerFrame->pack(-side => 'left', -anchor => 'w');
    
    my $entriesFrame = $mwf->Frame ();
    $entriesFrame->pack(-side => 'left', -anchor => 'w');

#  __    __   __   ____    _____   _____   ______   _____          
# |  |/\|  | |  | |    \  /  ___\ |  ___| |_    _| /  ___>         
# |        | |  | |  |  | |  \_ \ |  ___|   |  |   \___  \         
#  \__/\__/  |__| |____/  \_____/ |_____|   |__|   <_____/         
    my $dmxsFrame = $mwf->Frame ();
    $dmxsFrame->pack(-side => 'left', -anchor => 'w');
    generateOneServoFrame($dmxsFrame, 0);
    generateOneServoFrame($dmxsFrame, 1);
}

sub generateOneServoFrame ($$) {
    my ($frame, $escIdx) =@_;
 
    my $clinkFrame = $frame->Frame (-bd => '1m', -relief => 'sunken');
    $clinkFrame->pack(-side => 'left', -anchor => 'w');
  
    my $scalabframe1 = $clinkFrame->Frame()->pack(-side => 'left', -anchor => 'w') ;
    $scalabframe1->Label ('-text' => "CLINK ")->
	pack (-side => 'top', -anchor => 'n');

    $scalabframe1->Checkbutton(
        -text     => 'On',
	#	-state => 'disabled',
        -variable => \ ($tkObject{"clinkOn${escIdx}"}),
	-relief   => 'flat')->pack (-side => 'left', -anchor => 'n') ;
    
    $tkObject{"clinkScale${escIdx}"} = $scalabframe1->Scale (
	'-orient' => 'vertical', '-length' => 600, 
	'-from' => 100, '-to' => 0,
	'-resolution' => 0.5,
	'-variable' => \ ($tkObject{"clink${escIdx}"}),
	'-background' => 'lightgreen',
	'-sliderlength' => 20,
	'-sliderrelief' => 'solid');

    $tkObject{"clinkScale${escIdx}"}->pack (-side => 'top', -anchor => 'n') ;

    my $dataFrame = $frame->Frame (-bd => '1m', -relief => 'sunken');
    $dataFrame->pack(-side => 'left', -anchor => 'w');

    foreach my $varName (sort keys %{$varData[$escIdx]}) {
	labelLabelFrame($dataFrame, "$varName = ", \ ($varData[$escIdx]->{$varName}), 'left', 10);
    }

}



sub labelLabelFrame ($$$$;$)
{
    my ($ef, $labelText, $textVar, $packDir, $width) = @_ ;
    
    my (
	$label,
	$entry,
	$frame,
	$frameDir
	) = ();
    
    $frameDir = ($packDir eq 'top') ? 'left' : 'top' ; 
    
    $width = 15 unless defined $width ;
    $frame = $ef->Frame ();
    $frame->pack (-side => $frameDir, -pady => '2m', -padx => '0m', 
		  -anchor => 'w', -fill => 'both', -expand => 'true');
    
    $label = $frame->Label (-text => $labelText);
    $label->pack (-side =>$packDir, -padx => '0m', -fill => 'y');
    
    $entry = $frame->Label (-width => $width, -relief => 'sunken',
			    -bd => 2, -textvariable => $textVar,
			    -font => "-adobe-courier-medium-r-*-*-14-*-*-*-*-*-iso8859-15") ;
    
    $entry->pack (-side =>'right', -padx => '0m', -anchor => 'e');
    
    return $entry ;
}



#                 ______                 _            _          
#                /  ____|               (_)          | |         
#                | (___     ___   _ __   _     __ _  | |         
#                 \___ \   / _ \ | '__| | |   / _` | | |         
#                .____) | |  __/ | |    | |  | (_| | | |         
#                \_____/   \___| |_|    |_|   \__,_| |_|         



sub initSerial ($)
{
    my $dev = shift;
 
    my $port = tie (*FHD, 'Device::SerialPort', $dev);
    unless ($port) {
	warn "Can't tie: $! .. still trying\n"; 
	return undef;
    }

#port configuration 115200/8/N/1
    $port->databits(8);
    $port->baudrate(115200);
    $port->parity("none");
    $port->stopbits(1);
    $port->handshake("none");
    $port->buffers(1, 1); #1 byte or it buffers everything forever
    $port->write_settings           || undef $port; #set
    unless ($port)                  { die "couldn't write_settings"; }

    return $port;
}

use constant  WAIT_FOR_SYNC => 0;
use constant  WAIT_FOR_LEN => 1;
use constant  WAIT_FOR_PAYLOAD => 2;
use constant  WAIT_FOR_CHECKSUM => 3;

sub serialCb()
{
    state $state = WAIT_FOR_SYNC;
    state @sync;
    state $buffer;
    state $len;
    state $crcBuf;
    state @crc;
    state $calculatedCrc;
    state $receivedCrc;
    my $totLen;

    @sync = (0,0) unless @sync;
    for ($state) {
	when  (WAIT_FOR_SYNC) {
	    $sync[0] = $sync[1];
	    sysread (FHD, $buffer, 1);
	    $sync[1] = unpack ('C', $buffer);
	    if (($sync[0] == 0xED) && ($sync[1] == 0xFE)) {
		$state = WAIT_FOR_LEN;
	    } 
	}

	when  (WAIT_FOR_LEN) {
	    sysread (FHD, $buffer, 1);
	    ($len) = unpack ('C', $buffer);
	    $state = WAIT_FOR_PAYLOAD;  
	}
	
	when  (WAIT_FOR_PAYLOAD) {
#	    say ("len is $len");
	    $totLen = 0;
	    do {
		$totLen += sysread (FHD, $buffer, $len-$totLen, $totLen);
	    } while ($totLen != $len);
	    
	    $receivedCrc = fletcher16 (\$buffer, undef);
	    $state = WAIT_FOR_CHECKSUM;
	}
	
	when  (WAIT_FOR_CHECKSUM) {
	    $totLen = 0;
	    do {
		$totLen += sysread (FHD, $crcBuf, 2-$totLen, $totLen);
	    } while ($totLen != 2);
	    
	    @crc = unpack ('CC', $crcBuf);
	    $calculatedCrc =  ($crc[1] << 8)  | $crc[0];
	    if ($calculatedCrc == $receivedCrc) {
		castelLinkMessageCb (\$buffer);
	    } else {
		printf ("CRC DIFFER C:0x%x != R:0x%x\n", $calculatedCrc, $receivedCrc);
	    }
	    $state = WAIT_FOR_SYNC;
	}
    }
}


sub fletcher16 ($$) 
{
    my ($bufferRef, $msgIdRef) = @_;
    my $sum1 = 0; # 8 bits
    my $sum2 = 0; # 8 bits
    my $index;
    
    my @buffer = unpack ('C*', $$bufferRef);
    my $count = scalar(@buffer);
    
    $sum1 = ($sum1 + $count) % 0xff;
    $sum2 = $sum1;

    for($index = 0; $index < $count; $index++)  {
#	say "B[$index]=$buffer[$index]";
	$sum1 = ($sum1 + $buffer[$index]) % 0xff;
	$sum2 = ($sum2 + $sum1) % 0xff;
    }
    
    $$msgIdRef=$buffer[0] if defined $msgIdRef; # msgId
    return (($sum2 << 8) | $sum1);
}

sub castelLinkMessageCb ($)
{
    my ($bufferRef) = @_;
    my ($bat_voltage, $ripple_voltage, $current, $throttle, $power,		   
	$rpm, $bec_voltage, $bec_current, $temperature, $channel) = unpack ('f9L', $$bufferRef);


    $varData[$channel]->{'bat_voltage'} = sprintf ("%.2f", $bat_voltage);
    $varData[$channel]->{'ripple_voltage'} = sprintf ("%.2f", $ripple_voltage);
    $varData[$channel]->{'current'} = sprintf ("%.2f", $current);  
    $varData[$channel]->{'throttle'} = sprintf ("%.2f", $throttle);
    $varData[$channel]->{'power'} = sprintf ("%.2f", $power);	      
    $varData[$channel]->{'rpm'} = sprintf ("%.0f", $rpm);
    $varData[$channel]->{'bec_voltage'} = sprintf ("%.2f", $bec_voltage);    
    $varData[$channel]->{'bec_current'} = sprintf ("%.2f", $bec_current);
    $varData[$channel]->{'temperature'} = sprintf ("%.1f", $temperature);
#    say ".$channel";
}


sub simpleMsgSend ($)
{
    my $bufferRef = shift;
    my $len = length $$bufferRef;
    my $crc = fletcher16 ($bufferRef, undef);
    my @sync = (0xED, 0xFE);

    my $msgHeader = pack ('C3', @sync, $len);
    my $msgTrailer = pack ('S', $crc);
    unless (syswrite (FHD, $msgHeader)) {
	say "serial port write error";
	exit;
    }
    syswrite (FHD, $$bufferRef);
    syswrite (FHD, $msgTrailer);
}

sub getSerial()
{
    opendir (my $dhd, "/dev") || die "cannot opendir /dev\n";
    my @acm;

    while (my $fn = readdir ($dhd)) {
	push (@acm, $fn) if $fn =~ m|^stm_acm_\d+|;
    }
    closedir ($dhd);

    die "no ACM device\n" unless scalar @acm;
    return '/dev/' . (reverse sort (@acm))[0];
}


sub geneCastelLinkMsgsCB($)
{
    foreach my $escIdx (0,1) {
	my $pwmInTenThousand = 1000+($tkObject{"clink${escIdx}"}*10); 
	my $active = $tkObject{"clinkOn${escIdx}"}; # should get what have been select by radio button
	
	my $rout;
	while (select($rout=$selectReadBits, undef, undef, 0.001)) {
	    serialCb();
	}
	
	my $buffer = pack ('SSS', (0, $escIdx, $active ? $pwmInTenThousand: -1));
	simpleMsgSend(\$buffer);
	#	    say "DBG> [0, $escIdx, $pwmInTenThousand]"; 
    
    }
}


sub fhbits(@) 
{
    my @fhlist = @_;
    my $bits = "";
    for my $fh (@fhlist) {
	vec($bits, fileno($fh), 1) = 1;
    }
    return $bits;
}

