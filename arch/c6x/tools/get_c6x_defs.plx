#!/usr/bin/perl

$ASM_FILE=$ARGV[0];
$defs_file=$ARGV[1];
$found = 0;
$extra = "";

system "rm -f $defs_file";
system "touch $defs_file";

open (INFILE, $ASM_FILE);
while (<INFILE>) {
  if ($_ =~ m/\.stag._task_struct/) {
    $found = 1;
    $extra = "";
    next;
  }
  elsif ($_ =~ m/\.stag.task_struct/) {
    $found = 1;
    $extra = "_";
    next;
  }
  if (($found) && ($_ =~ m/.eos/)) {
    $found = 2;
  }

  if (($found == 1) && ($_ =~ m/\.member.(\w+),.(\w+)/)) {
    ($name = $1) =~ tr/a-z/A-Z/;
    ($offset = $2) /= 8;
    system "echo 'TASK$extra$name\t.set\t$offset' >> $defs_file";
  }

  next;
}

open (INFILE, $ASM_FILE);
while (<INFILE>) {
  if ($_ =~ m/\.stag._thread_info/) {
    $found = 1;
    $extra = "";
    next;
  }
  elsif ($_ =~ m/\.stag.thread_info/) {
    $found = 1;
    $extra = "_";
    next;
  }
  if (($found) && ($_ =~ m/.eos/)) {
    $found = 2;
  }

  if (($found == 1) && ($_ =~ m/\.member.(\w+),.(\w+)/)) {
    ($name = $1) =~ tr/a-z/A-Z/;
    ($offset = $2) /= 8;
    system "echo 'THREAD_INFO$extra$name\t.set\t$offset' >> $defs_file";
  }

  next;
}

open (INFILE, $ASM_FILE);
while (<INFILE>) {
  if ($_ =~ m/\.stag._thread_struct/) {
    $found = 1;
    $extra = "";
    next;
  }
  elsif ($_ =~ m/\.stag.thread_struct/) {
    $found = 1;
    $extra = "_";
    next;
  }
  if (($found) && ($_ =~ m/.eos/)) {
    $found = 2;
  }

  if (($found == 1) && ($_ =~ m/\.member.(\w+),.(\w+)/)) {
    ($name = $1) =~ tr/a-z/A-Z/;
    ($offset = $2) /= 8;
    system "echo 'THREAD$extra$name\t.set\t$offset' >> $defs_file";
  }

  next;
}

open (INFILE, $ASM_FILE);
while (<INFILE>) {
  if ($_ =~ m/\.stag._pt_regs/) {
    $found = 1;
    $extra = "";
    next;
  }
  elsif ($_ =~ m/\.stag.pt_regs/) {
    $found = 1;
    $extra = "_";
    next;
  }
  if (($found) && ($_ =~ m/.eos/)) {
    $found = 2;
  }

  if (($found == 1) && ($_ =~ m/\.member.(\w+),.(\w+)/)) {
    ($name = $1) =~ tr/a-z/A-Z/;
    ($offset = $2) /= 8;
    system "echo 'REGS$extra$name\t.set\t$offset' >> $defs_file";
  }

  next;
}

open (INFILE, $ASM_FILE);
while (<INFILE>) {
  if ($_ =~ m/\.stag._NkOsCtx/) {
    $found = 1;
    $extra = "";
    next;
  }
  elsif ($_ =~ m/\.stag.NkOsCtx/) {
    $found = 1;
    $extra = "_";
    next;
  }
  if (($found) && ($_ =~ m/.eos/)) {
    $found = 2;
  }

  if (($found == 1) && ($_ =~ m/\.member.(\w+),.(\w+)/)) {
    ($name = $1) =~ tr/a-z/A-Z/;
    ($offset = $2) /= 8;
    system "echo 'NKCTX$extra$name\t.set\t$offset' >> $defs_file";
  }

  next;
}
