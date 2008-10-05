package Zaptel::Xpp::Line;
#
# Written by Oron Peled <oron@actcom.co.il>
# Copyright (C) 2008, Xorcom
# This program is free software; you can redistribute and/or
# modify it under the same terms as Perl itself.
#
# $Id: Line.pm 4039 2008-03-21 01:51:39Z tzafrir $
#
use strict;
use Zaptel::Utils;

my $proc_base = "/proc/xpp";

sub new($$$) {
	my $pack = shift or die "Wasn't called as a class method\n";
	my $xpd = shift or die;
	my $index = shift;
	defined $index or die;
	my $self = {};
	bless $self, ref($xpd);
	$self->{XPD} = $xpd;
	$self->{INDEX} = $index;
	return $self;
}

sub create_all($$) {
	my $pack = shift or die "Wasn't called as a class method\n";
	my $xpd = shift || die;
	my $procdir = shift || die;
	local $/ = "\n";
	my @lines;
	for(my $i = 0; $i < $xpd->{CHANNELS}; $i++) {
		my $line = Zaptel::Xpp::Line->new($xpd, $i);
		push(@lines, $line);
	}
	$xpd->{LINES} = \@lines;
	my ($infofile) = glob "$procdir/*_info";
	die "Failed globbing '$procdir/*_info'" unless defined $infofile;
	my $type = $xpd->type;
	open(F, "$infofile") || die "Failed opening '$infofile': $!";
	my $battery_info = 0;
	while (<F>) {
		chomp;
		if($type eq 'FXO') {
			$battery_info = 1 if /^Battery:/;
			if($battery_info && s/^\s*on\s*:\s*//) {
				my @batt = split;
				foreach my $l (@lines) {
					die unless @batt;
					my $state = shift @batt;
					$l->{BATTERY} = ($state eq '+') ? 1 : 0;
				}
				$battery_info = 0;
				die if @batt;
			}
		}
	}
	close F;
}


1;
