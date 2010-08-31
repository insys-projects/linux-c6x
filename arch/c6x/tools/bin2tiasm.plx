#!/usr/bin/perl -w

use File::Basename qw(dirname basename);
use File::stat;
use bytes;
use strict;

my $section_name = ".data";
my $length     = 0;
my $output_file = "";

my @file_list = ();

# Process args
my $arg;
foreach $arg (@ARGV) {

  if ($arg =~ m/--section-name=/) {
    $arg =~ s,--section-name=,,;
    $section_name = $arg;
    next;
  }

  if ($arg =~ m/--output-file=/) {
    $arg =~ s,--output-file=,,;
    $output_file = $arg;
    next;
  }

  # else an input file
  push @file_list, $arg;
}

my $file;
foreach $file (@file_list) {
  my $fh_in;
  my $fh_out;
  my $buf;
  my $size;
  my $outfile = $file . ".S64";

  $outfile = $output_file unless ($output_file eq "");

  open($fh_in, "< $file") or die("cannot open file '$file'\n");
  $size = (stat($fh_in))->size;
  $size = min($size, $length) if ($length > 0);

  open($fh_out, "> $outfile") or die("cannot open file $outfile\n");

  printf $fh_out "; auto generated from %s\n", $file;
  printf $fh_out "\t.sect \"%s\",ALLOC\n", $section_name;

  for (; $size > 0; $size--) {
    read($fh_in, $buf, 1);
    printf $fh_out "\t.byte 0%2.2Xh\n", ord($buf);
  }

  close($fh_in);
  close($fh_out);
}

