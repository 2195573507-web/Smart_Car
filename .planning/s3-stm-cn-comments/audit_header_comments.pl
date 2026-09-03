#!/usr/bin/env perl

use strict;
use warnings;

my $checked_count = 0;
my $missing_count = 0;

for my $path (@ARGV) {
    open my $handle, '<', $path or die "cannot read $path: $!\n";
    my @lines = <$handle>;
    close $handle;
    my %complete_declarations;

    for (my $end = 0; $end < @lines; ++$end) {
        next unless $lines[$end] =~ /\)\s*(?:__attribute__\s*\(\(.*\)\)\s*)?;\s*$/;

        my $start = $end;
        while ($start > 0) {
            my $previous = $lines[$start - 1];
            last if $previous =~ /^\s*$/;
            last if $previous =~ /\*\/\s*$/;
            last if $previous =~ /[;{}]\s*$/;
            last if $previous =~ /^\s*#/;
            --$start;
        }

        my $signature = join '', @lines[$start .. $end];
        next if $signature =~ /\b(?:return|_Static_assert|_Alignas)\b/;
        next if $signature =~ /^\s*(?:if|for|while|switch)\b/;
        next if $signature =~ /=/;

        my $name;
        if ($signature =~ /\btypedef\b/) {
            ($name) = $signature =~ /\(\*\s*([A-Za-z_]\w*)\s*\)/s;
        } else {
            ($name) = $signature =~
                /^\s*(?:extern\s+)?(?:static\s+)?(?:inline\s+)?(?:const\s+)?
                 (?:struct\s+\w+\s+|enum\s+\w+\s+|[A-Za-z_]\w*)
                 [A-Za-z0-9_\s*]*?\b([A-Za-z_]\w*)\s*\(/sx;
        }
        next unless defined $name;
        ++$checked_count;

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

        my @missing;
        push @missing, 'doxygen' unless $comment =~ /^\s*\/\*\*/;
        push @missing, 'author' unless $comment =~ /\@author/;
        push @missing, 'date' unless $comment =~ /\@date/;
        push @missing, 'parameter' unless
            $comment =~ /\@param|传入参数/;
        push @missing, 'return' unless $comment =~ /\@return|返回值/;
        push @missing, 'call' unless $comment =~ /调用方式/;
        push @missing, 'thread' unless $comment =~ /线程约束/;

        if (@missing) {
            ++$missing_count;
            my $line_number = $start + 1;
            print "$path:$line_number:$name missing=", join(',', @missing), "\n";
        } else {
            $complete_declarations{$name} = 1;
        }
    }

    # BSD ctags 不枚举头文件内联定义；在头文件层直接补充 static inline 覆盖。
    for (my $start = 0; $start < @lines; ++$start) {
        next unless $lines[$start] =~ /^\s*static\s+inline\b/;

        my $end = $start;
        ++$end while $end < @lines && $lines[$end] !~ /\{/;
        next if $end >= @lines;

        my $signature = join '', @lines[$start .. $end];
        my ($name) = $signature =~ /\b([A-Za-z_]\w*)\s*\(/s;
        next unless defined $name;
        ++$checked_count;

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

        my @missing;
        push @missing, 'doxygen' unless $comment =~ /^\s*\/\*\*/;
        push @missing, 'author' unless $comment =~ /\@author/;
        push @missing, 'date' unless $comment =~ /\@date/;

        if ($comment =~ /\@copydoc\s+([A-Za-z_]\w*)/) {
            my $target = $1;
            push @missing, 'copydoc_target'
                unless $complete_declarations{$target};
        } else {
            push @missing, 'brief' unless $comment =~ /\@brief/;
            push @missing, 'parameter' unless
                $comment =~ /\@param|传入参数/;
            push @missing, 'return' unless $comment =~ /\@return|返回值/;
            push @missing, 'call' unless $comment =~ /调用方式/;
            push @missing, 'thread' unless $comment =~ /线程约束/;
        }

        if (@missing) {
            ++$missing_count;
            my $line_number = $start + 1;
            print "$path:$line_number:$name missing=", join(',', @missing), "\n";
        }
    }
}

if ($ENV{AUDIT_SUMMARY}) {
    print "checked=$checked_count missing=$missing_count\n";
}
