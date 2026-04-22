proc _selftimekeep_find_proj_dir {start_dir} {
    set d [file normalize $start_dir]
    for {set i 0} {$i < 10} {incr i} {
        if {[file exists [file join $d sim run_fifo.do]]} {
            return $d
        }
        set parent [file dirname $d]
        if {$parent eq $d} {
            break
        }
        set d $parent
    }
    return ""
}

set SCRIPT_FILE ""
set MAX_LVL 0
catch {set MAX_LVL [info level]}
for {set lvl $MAX_LVL} {$lvl >= 0} {incr lvl -1} {
    if {[catch {set fr [info frame $lvl]}]} {
        continue
    }
    if {[dict exists $fr file]} {
        set f [dict get $fr file]
        if {$f ne "" && [file exists $f]} {
            set SCRIPT_FILE $f
            break
        }
    }
}
if {$SCRIPT_FILE eq ""} {
    catch {set SCRIPT_FILE [info script]}
}
if {$SCRIPT_FILE eq ""} {
    if {[info exists ::argv0] && $::argv0 ne "" && [file exists $::argv0]} {
        set SCRIPT_FILE $::argv0
    }
}

set PROJ_DIR ""
if {[info exists ::SELF_TIMEKEEP_PROJ_DIR] && $::SELF_TIMEKEEP_PROJ_DIR ne ""} {
    set PROJ_DIR [file normalize $::SELF_TIMEKEEP_PROJ_DIR]
} elseif {$SCRIPT_FILE ne ""} {
    set PROJ_DIR [_selftimekeep_find_proj_dir [file dirname $SCRIPT_FILE]]
}
if {$PROJ_DIR eq ""} {
    set PROJ_DIR [_selftimekeep_find_proj_dir [pwd]]
}
if {$PROJ_DIR eq ""} {
    set FALLBACK_DIR [file normalize "E:/fjlwork/selftimekeep"]
    if {[file exists [file join $FALLBACK_DIR sim run_fifo.do]]} {
        set PROJ_DIR $FALLBACK_DIR
    }
}
if {$PROJ_DIR eq ""} {
    set PROJ_DIR [file normalize [pwd]]
}
set ::SELF_TIMEKEEP_PROJ_DIR $PROJ_DIR
if {![info exists ::ANLOGIC_SIM_LIB_ROOT] || $::ANLOGIC_SIM_LIB_ROOT eq ""} {
    set ::ANLOGIC_SIM_LIB_ROOT "E:/TD_6.2.1/sim_release/ph1"
}
do [file join $PROJ_DIR sim run_fifo.do]
