#!/usr/bin/env perl

use strict;
use warnings;

my @paths = @ARGV;
die "usage: $0 <source.c>...\n" unless @paths;

my $checked_count = 0;
my $missing_count = 0;
my $name_pattern = $ENV{AUDIT_NAME_PATTERN};

for my $source_path (@paths) {
    open my $tags, '-|', 'ctags', '-x', $source_path
        or die "cannot run ctags for $source_path: $!\n";

    while (my $entry = <$tags>) {
        chomp $entry;
        next unless $entry =~ /^(\S+)\s+(\d+)\s+(\S+)\s+(.+)$/;

        my ($name, $line_number, $path, $signature) = ($1, $2, $3, $4);
        next unless $signature =~ /\(/;
        next if $ENV{AUDIT_STATIC_ONLY} && $signature !~ /^static\b/;
        next if defined $name_pattern && length $name_pattern &&
                $name !~ /$name_pattern/;

        open my $handle, '<', $path or die "cannot read $path: $!\n";
        my @lines = <$handle>;
        close $handle;

        my $start = $line_number - 1;
        while ($start > 0) {
            my $previous = $lines[$start - 1];
            last if $previous =~ /^\s*$/;
            last if $previous =~ /\*\/\s*$/;
            last if $previous =~ /[;{}]\s*$/;
            last if $previous =~ /^\s*#/;
            --$start;
        }

        my $comment_end = $start - 1;
        while ($comment_end >= 0 && $lines[$comment_end] =~ /^\s*$/) {
            --$comment_end;
        }

        my $comment = '';
        if ($comment_end >= 0 && $lines[$comment_end] =~ /\*\/\s*$/) {
            my $comment_start = $comment_end;
            while ($comment_start >= 0 &&
                   $lines[$comment_start] !~ /^\s*\/\*\*/) {
                --$comment_start;
            }
            if ($comment_start >= 0) {
                $comment = join '', @lines[$comment_start .. $comment_end];
            }
        }

        ++$checked_count;
        my @missing;
        push @missing, 'doxygen' unless $comment =~ /^\s*\/\*\*/;
        push @missing, 'brief' unless $comment =~ /\@brief/;
        push @missing, 'author' unless $comment =~ /\@author/;
        push @missing, 'date' unless $comment =~ /\@date/;
        push @missing, 'parameter' unless $comment =~ /\@param|传入参数/;
        push @missing, 'return' unless $comment =~ /\@return|返回值/;
        push @missing, 'call' unless $comment =~ /调用方式/;
        push @missing, 'thread' unless $comment =~ /线程约束/;

        if (@missing) {
            ++$missing_count;
            print "$path:$line_number:$name missing=", join(',', @missing), "\n";
        }
    }
    close $tags;
}

if ($ENV{AUDIT_SUMMARY}) {
    print "checked=$checked_count missing=$missing_count\n";
}
