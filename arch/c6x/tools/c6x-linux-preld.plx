#!/usr/bin/perl -w

use File::Basename qw(dirname basename);
use File::stat;
use bytes;

$debug = $ENV{"C6X_PRELD_DEBUG"}+0;

$final_file = "";
$not_to_be_taken = 0;
$final_link = 0;
$final_script = "";
@original_args = ();
@file_list = ();
@sect_list1 = ();
@sect_list2 = ();
@sect_list3 = ();
@sect_list4 = ();
@sect_list5 = ();
@sect_list6 = ();
@sect_list7 = ();
$sect = "";
$tmp_file;
$tmp_cmd;

use constant COFFMAG => "\xc2\x00";
use constant S_COFFMAG => length(COFFMAG);
use constant S_HEADER => 22;
use constant S_SYMBENT => 18;
use constant S_SECHEADER => 48;

sub mk_tempname {
  my ($fn, $suffix, $pass) = @_;
  $fn =~ s!/!_!g;
  return "__c6x_tmp_preld_" . $pass . "_" . $fn . "_" . $$ . $suffix;
}

sub read_at {
  my ($fn, $fh, $off, $sz, $buf) = @_;

  seek($fh, $off, 0) or
    die("cannot seek at offset $off file '$fn': $!\n");

  read($fh, $$buf, $sz, 0) or
    die("cannot read $sz bytes of file '$fn': $!\n");
}

sub find_section {
  my ($fn, $pattern) = @_;
  my $fh;
  my $buf;
  my @fields;
  my $hdr;
  my $size;
  my $strings;
  my $name = "";

  open($fh, "< $fn") or die("cannot open file '$fn': $!\n");
  binmode($fh);
  read_at($fn, $fh, 0, S_COFFMAG, \$buf);

  if (substr($buf, 0, S_COFFMAG) ne COFFMAG) {
    return $name;
  }

  $size = (stat($fh))->size;

  read_at($fn, $fh, S_COFFMAG, S_HEADER - S_COFFMAG, \$buf);
  @fields = unpack("vVVVvvv", $buf);
  @$hdr{qw/secnb stamp symboff symbnb opthdrsz flags id/} = @fields;

  $stroff = $hdr->{symboff} + $hdr->{symbnb} * S_SYMBENT;
  $strsz = $size - $stroff;

  read_at($fn, $fh, $stroff, $strsz, \$strings);

  for (my $sec = 0; $sec < $hdr->{secnb}; $sec++) {
    my $off = S_HEADER + $hdr->{opthdrsz} + ($sec * S_SECHEADER);
    read_at($fn, $fh, $off, 8, \$buf);
    $name = unpack("Z8", $buf);
    if (length($name) == 0) {
      $off = unpack("x4V", $buf);
      $name = unpack("x".$off."Z*", $strings);
    }
    if ($name =~ m/$pattern/) {
      last;
    }
    $name = "";
  }

  close($fh);

  return $name;
}

sub is_empty_archive {
  my ($fn) = @_;
  my ($fh);
  my ($buf, $dummy);
  my ($rlen, $eof);

  open($fh, "< $fn") or die("cannot open file '$fn': $!\n"); 
  binmode($fh);
  $rlen = read($fh, $buf, 8);
  $eof  = (read($fh, $dummy, 1) == 0);
  close($fh);
#  print "rlen=$rlen eof=$eof buf=$buf\n";
  return ($rlen == 8 && $buf == "!<arch>\n" && $eof) 
}

sub mk_empty_archive {
  my ($fn) = @_;
  my $fh;
  my ($buf, $dummy);
  my ($rlen, $eof);

  open($fh, "> $fn") or die("cannot open file '$fn': $!\n"); 
  binmode($fh);
  print $fh "!<arch>\n";
  close($fh);
  return 0; 
}

sub unique_id {
  my ($filename) = @_;
  return unpack("%16Z*", $filename);
}

# Link file by file
foreach $arg (@ARGV) {

  if ($arg =~ m/--final-script/) {
    $final_link = 1;
    ($final_script = $arg) =~ s,--final-script=,,;
    next;
  }

  if ($arg =~ m/\-o/) {
    $not_to_be_taken = 1;
    push @original_args, $arg;
    next;
  }

  if ($not_to_be_taken == 1) {
    $final_file = $arg;
    $not_to_be_taken = 0;
    push @original_args, $arg;
    next;
  }

  if ($arg =~ m/\.o/) {
    if (is_empty_archive($arg)) {
      $empty_seen = 1;
      print "PRELD:skipping empty archive $arg\n" if $debug;
      next;
    }     
    push @file_list, $arg;
  }
  push @original_args, $arg;
}

if ($empty_seen && $final_file && scalar(@file_list) <= 0) {
  my $rc;

  # All the input objects are empty, 
  # there may be libraries but w/o .o's nothing will pull anything in
  # just create a null archive as the output
  print "PRELD: creating null archive $final_file\n" if $debug;
  $rc = mk_empty_archive($final_file); 
  exit($rc);
}

foreach $file (@file_list) {
  my $fh;
  my $buffer;
  my $sect_name;
  my $level;
  my $sect_found;

  $tmp_cmd = mk_tempname($file, ".cmd", "fixup");
  open($fh, "> $tmp_cmd") or
    die("cannot open archive $tmp_cmd: $!");

  $sect_found = 0;
  $buffer  = "SECTIONS\n{\n";

  for ($level = 1; $level < 8; $level += 1) {
    $sect = find_section($file, "\.initcall$level.*");

    if ($sect eq "") {
      next;
    }

    $sect_name = $sect;
    if ($sect eq ".initcall$level") {
      $sect_found = 1;
      $sect_name = $sect . "_" . basename($file) . "_" . unique_id($file);
      $sect_name =~ s/\-/\_/g;
      $buffer .= "   $sect_name  { *($sect) }\n";
    }

    $sect = "sect_list$level";
    push @$sect, $sect_name;
  }

  if ($sect_found == 0) {
    close($fh);
    unlink($tmp_cmd) unless $debug;
    next;
  }
  
  $buffer .= "}\n";
  print($fh $buffer);
  close($fh);

  $tmp_file = mk_tempname($file, ".o", "fixup");
  rename $file,$tmp_file;
  print "PRELD: 1st pass cmd=", "$ARGV[0] ","-nostdlib ","-r ","-Wl,-qq ","$tmp_cmd ","$tmp_file ","-o ","$file ","\n" if $debug;
  system "$ARGV[0]","-nostdlib","-r","-Wl,-qq","$tmp_cmd","$tmp_file","-o","$file";
  unlink($tmp_file) unless $debug;
  unlink($tmp_cmd) unless $debug;
}

$tmp_cmd = mk_tempname($final_file, ".cmd", $final_link ? "final" : "mid");
open($fh, "> $tmp_cmd") or
  die("cannot open archive $tmp_cmd: $!");

if ($final_link == 1)
{
  # Final link
  my $sub_buffer1 = "";
  my $sub_buffer2 = "";
  my $sub_buffer3 = "";
  my $sub_buffer4 = "";
  my $sub_buffer5 = "";
  my $sub_buffer6 = "";
  my $sub_buffer7 = "";
  my $level;
  my $sect_list_name;
  my $sub_buffer_name;

  for ($level = 1; $level < 8; $level += 1) {
    $sect_list_name = "sect_list$level";
    $sub_buffer_name = "sub_buffer$level";
    foreach $sect (@$sect_list_name) {
      $$sub_buffer_name .= "   *(" . $sect . ")\n";
    }
  }

  $level = 1;

  open(INFILE, "$final_script");
  while (<INFILE>) {
    if ($_ =~ m/\@INITCALLS/) {
      $sub_buffer_name = "sub_buffer$level";
      $buffer .= $$sub_buffer_name;
      $level += 1;
    } else {
      $buffer .= $_;
    }
  }
  close(INFILE);

  print($fh $buffer);
  close($fh);

  print "PRELD: final link cmd=", join(" ", @original_args), " $tmp_cmd\n" if $debug;
  $ret = system @original_args, "$tmp_cmd";
  unlink($tmp_cmd) unless $debug;
  exit($ret>>8);
}

# Intermediate link

$sect_found = 0;
$buffer  = "SECTIONS\n{\n";

for ($level = 1; $level < 8; $level += 1) {
  $sect_list_name = "sect_list$level";
  $list_length = @$sect_list_name;

  if ($list_length == 0) {
    next;
  }

  $sect_found = 1;
  $buffer .= "   .initcall$level {\n";
  foreach $sect (@$sect_list_name) {
    $buffer .= "   *(" . $sect . ")\n";
  }
  $buffer .= "   }\n";
}

$buffer .= "}\n";

print($fh $buffer);
close($fh);

print "PRELD: 2nd pass cmd=", join(" ", @original_args), " $tmp_cmd\n" if $debug;
if ($sect_found == 1) {
  $ret = system @original_args, "$tmp_cmd";
} else {
  $ret = system @original_args;
}

unlink($tmp_cmd) unless $debug;
exit($ret>>8);

