#!/usr/bin/env perl

use strict;
use warnings;
use utf8;

sub preceding_comment {
    my ($lines, $index) = @_;

    while ($index >= 0 && $lines->[$index] =~ /^\s*$/) {
        --$index;
    }
    return '' if $index < 0;

    if ($lines->[$index] =~ /\*\/\s*$/) {
        my $start = $index;
        while ($start >= 0 && $lines->[$start] !~ /\/\*/) {
            --$start;
        }
        return $start >= 0 ? join('', @$lines[$start .. $index]) : '';
    }
    if ($lines->[$index] =~ /^\s*\/\//) {
        my $start = $index;
        while ($start > 0 && $lines->[$start - 1] =~ /^\s*\/\//) {
            --$start;
        }
        return join('', @$lines[$start .. $index]);
    }
    return '';
}

sub has_chinese_comment {
    my ($text) = @_;
    return $text =~ /\p{Han}/ ? 1 : 0;
}

sub same_line_comment {
    my ($line) = @_;
    return '' unless $line =~ m{(//.*|/\*.*\*/)};
    return $1;
}

my $type_count = 0;
my $type_missing = 0;
my $member_count = 0;
my $member_missing = 0;

for my $path (@ARGV) {
    open my $handle, '<:encoding(UTF-8)', $path
        or die "cannot read $path: $!\n";
    my @lines = <$handle>;
    close $handle;

    for (my $index = 0; $index < @lines; ++$index) {
        next unless $lines[$index] =~ /^\s*typedef\s+(struct|enum)\b/;
        my $kind = $1;
        my $start = $index;
        my $open = $index;
        ++$open while $open < @lines && $lines[$open] !~ /\{/;
        next if $open >= @lines;

        my $close = $open + 1;
        ++$close while $close < @lines &&
            $lines[$close] !~ /^\s*}\s*(?:__attribute__\s*\(\([^;]*\)\)\s*)?([A-Za-z_]\w*)\s*;/;
        next if $close >= @lines;
        my ($name) = $lines[$close] =~
            /^\s*}\s*(?:__attribute__\s*\(\([^;]*\)\)\s*)?([A-Za-z_]\w*)\s*;/;
        next unless defined $name;

        ++$type_count;
        my $type_comment = preceding_comment(\@lines, $start - 1);
        if (!has_chinese_comment($type_comment)) {
            ++$type_missing;
            print "$path:", ($start + 1), ":type:$name missing=chinese_comment\n";
        }

        if ($ENV{AUDIT_MEMBERS}) {
            if ($kind eq 'enum') {
                for my $member_line ($open + 1 .. $close - 1) {
                    my $line = $lines[$member_line];
                    next if $line =~ /^\s*(?:#|\/\*|\*|\/\/|$)/;
                    next unless $line =~ /^\s*([A-Za-z_]\w*)\b/;
                    my $member = $1;
                    ++$member_count;
                    my $comment = same_line_comment($line);
                    $comment = preceding_comment(\@lines, $member_line - 1)
                        unless has_chinese_comment($comment);
                    if (!has_chinese_comment($comment)) {
                        ++$member_missing;
                        print "$path:", ($member_line + 1),
                              ":member:$name\::$member missing=chinese_comment\n";
                    }
                }
            } else {
                my $statement_start;
                for my $member_line ($open + 1 .. $close - 1) {
                    my $line = $lines[$member_line];
                    next if $line =~ /^\s*(?:#|\/\*|\*|\/\/|$)/ &&
                            !defined $statement_start;
                    $statement_start = $member_line unless defined $statement_start;
                    next unless $line =~ /;/;

                    my $statement = join('', @lines[$statement_start .. $member_line]);
                    my $label = $statement;
                    $label =~ s{/\*.*?\*/}{}gs;
                    $label =~ s{//.*$}{}gm;
                    $label =~ s/\s+/ /g;
                    $label =~ s/^\s+|\s+$//g;
                    ++$member_count;
                    my $comment = same_line_comment($line);
                    $comment = preceding_comment(\@lines, $statement_start - 1)
                        unless has_chinese_comment($comment);
                    if (!has_chinese_comment($comment)) {
                        ++$member_missing;
                        print "$path:", ($statement_start + 1),
                              ":member:$name\::$label missing=chinese_comment\n";
                    }
                    undef $statement_start;
                }
            }
        }
        $index = $close;
    }
}

if ($ENV{AUDIT_SUMMARY}) {
    print "types=$type_count type_missing=$type_missing";
    if ($ENV{AUDIT_MEMBERS}) {
        print " members=$member_count member_missing=$member_missing";
    }
    print "\n";
}
