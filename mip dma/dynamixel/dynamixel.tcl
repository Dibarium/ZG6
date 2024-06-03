#! tclsh

package require Tk

############################################################################
# Callbacks
############################################################################
proc ping_cb {} {
	puts "broadcast ping"
	puts -nonewline $::serial "X"
	flush $::serial
}

proc reset_cb {} {
	puts "factory reset!"
	puts $::serial "R$::id"
	flush $::serial
}

proc idfilter_cb {val} {
	if {$val < 0 || $val > 253} {
		set ::id  1 
	}
	return 1
}

proc idsetup_cb {} {
	puts "setting ID ..."
	puts $::serial "I$::id;$::nid"
	flush $::serial
}

proc test_cb {} {
	# enable/disable commands according to selected item
	puts "change test"
	switch $::test {
		1 {
			.nb.test.blktest configure -state normal
			.nb.test.plbl configure -state disabled
			.nb.test.pval configure -state disabled
			.nb.test.pset configure -state disabled
			.nb.test.vlbl configure -state disabled
			.nb.test.vval configure -state disabled
			.nb.test.vset configure -state disabled
		}
		2 {
			.nb.test.blktest configure -state disabled
			.nb.test.plbl configure -state normal
			.nb.test.pval configure -state normal
			.nb.test.pset configure -state normal
			.nb.test.vlbl configure -state disabled
			.nb.test.vval configure -state disabled
			.nb.test.vset configure -state disabled
		}
		3 {
			.nb.test.blktest configure -state disabled
			.nb.test.plbl configure -state disabled
			.nb.test.pval configure -state disabled
			.nb.test.pset configure -state disabled
			.nb.test.vlbl configure -state normal
			.nb.test.vval configure -state normal
			.nb.test.vset configure -state normal
		}
	}
}

proc blink_cb {} {
	puts "start blink test"
	puts $::serial "B$::id"
	flush $::serial
}

proc setpos_cb {} {
	puts "set position"
	puts $::serial "P$::id;$::pos"
	flush $::serial
}

proc setvel_cb {} {
	puts "set velocity"
	puts $::serial "V$::id;$::vel"
	flush $::serial
}

############################################################################
# GUI Variables / Robot parameters 
############################################################################
# gui status
set defstatus	"Dynamixel control ..."
set status		$defstatus

set id			1
set idlist		{1}

set nid			1

set test		1

set pos			0
set vel			0

set reloadimg [image create photo -format GIF -data {
	R0lGODlhGAAYAIYAAPwCBBRSDKS+nLTCrBxWFKzCpIy6fJS6hIy2hIy6hIS2fHyudHyqdCROHAw2
	VAwmBFyGVFySVGyiZGyqbHyydFyaVCRKHLTe7Ex2RHSubFSOTBw+FIzG3ITC1Ex+RERyPEyGRFya
	XGSmXIS+1IzC3IS+3EyOTFymXGyqZBQ2DHS2zHy21DRqNEyKRHSuzAwqBDx2PESaRFSiVBQ+ZGyq
	xGSixFyavARCZBRSFDyCPDyKPESKRFSaVEyeTEyWTCRiHAQuPGSivFSStDR2nDyaRESWRESaTDyS
	PDSKNESGrFSOrER6lBxWdCxuLDyWPBx6JDRulFSSvDRmhBxWfCRuJCyONAwuREyKtDRylDRqjCSK
	LCRqJCxijCxihDx+pCxqlCxqjAwyPCxmhDR2pDx2nCRejCxunDRynCRehCRmlBQ2RAAAAAAAAAAA
	AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAACH5
	BAEAAAAALAAAAAAYABgAAAf/gACCg4SFhoeEAYqKiI2KAgIDAwIEBI2FAQUDBgcIBwYCCJaXAAED
	CQoICwoKBqwMDZeZCg4PEBESE6sUChUWjhQOFxcPGBAQFRIUGRQaG4cBE8IcHB0PHh8gISIZGRIP
	0MIj4yQlDx8eJhUnKCcfKYbCKivz9Q8sICYhJyIt4ITCXAgcOPAFDAgxTsioAA+ggxkcaEisYeMG
	jhw6dvDoEcPHD0aEgLiwUSOIkCEPchAp4iNGDyNHkOT4NwhIEhtCbChZwuRFEyROgjrJ8YTmIAdQ
	hEQRclLKlAdUqkitUtTKIQdXkiTJiiULkwc/tGjZ8oALkKtdvGhV+wXM0ydFQ5mEaeRAzJAxXsaQ
	AVPm6wMmYs42AiJmzJAhZs58QYMGsOBLQNCYSTN5chomakjVBDIFTRrGQB5rFhS69OjTqFHLCAQA
	Ow==
}]

############################################################################
# GUI building
############################################################################
wm protocol . WM_DELETE_WINDOW exit
wm title . "Dynamixel motor control"

#grid [ttk::notebook .nb -takefocus 0]
pack [ttk::notebook .nb -takefocus 0] -fill x -side top

### setup page ###############################################
set w [frame .nb.setup]
#.nb add [frame .nb.setup] -text "Motor setup"
#grid [button $w.ping -text "\u267D" -command {ping_cb}] -row 0 -column 0 -sticky nwse -pady 5 -padx 5
grid [button $w.ping -image $reloadimg -command {ping_cb}] -row 0 -column 0 -sticky nwse -pady 5 -padx 5
grid [label $w.idlabel -text "current motor ID"]  -row 0 -column 1 -columnspan 3 -sticky e -pady 5 -padx 5
tk_optionMenu $w.idlist id {*}$idlist
grid $w.idlist -row 0 -column 4 -sticky nswe -pady 5 -padx 5
grid [button $w.reset -text "factory reset" -command {reset_cb}] -row 1 -column 0 -columnspan 5 -sticky nswe -pady 5 -padx 5
grid [label $w.nidlabel -text "new motor ID"] -row 2 -column 0 -columnspan 3 -sticky e -pady 5 -padx 5
grid [spinbox $w.id -width 6 -textvariable nid -from 0 -to 253 -increment 1 -validate key -validatecommand {idfilter_cb %P} -repeatdelay 1000 -repeatinterval 500] -row 2 -column 3 -sticky nwse -pady 5
grid [button $w.set -text "set" -command {idsetup_cb}] -row 2 -column 4 -sticky nwse -pady 5 -padx 5
grid columnconfigure $w "all" -minsize 10 -uniform a -weight 1
#grid rowconfigure $w "all" -minsize 8 -uniform a -weight 1
bind $w.id <Return> {idsetup_cb}
.nb add $w -text "Motor setup"

### test page ################################################
set w [frame .nb.test]
grid [button $w.ping -image $reloadimg -command {ping_cb}] -row 0 -column 0 -sticky nwse -pady 5 -padx 5
grid [label $w.idlabel -text "current motor ID"]  -row 0 -column 1 -columnspan 3 -sticky e -pady 5 -padx 5
tk_optionMenu $w.idlist id {*}$idlist
grid $w.idlist -row 0 -column 4 -sticky nswe -pady 5 -padx 5
grid [radiobutton $w.blink -text "Blink test" -variable test -value 1 -command test_cb] -row 1 -column 0 -columnspan 3 -sticky w
grid [button $w.blktest -text "Blink" -command blink_cb] -row 1 -column 4 -sticky nwse -pady 5 -padx 5
grid [radiobutton $w.postest -text "Position test" -variable test -value 2 -command test_cb] -row 2 -column 0 -columnspan 3 -sticky w
grid [label $w.plbl -text "position (0..4095)" -state disabled] -row 3 -column 1 -columnspan 2 -sticky e
grid [spinbox $w.pval -width 6 -state disabled -textvariable pos -from 0 -to 4095 -increment 50 -repeatdelay 1000 -repeatinterval 500] -row 3 -column 3 -sticky nswe -pady 5 -padx 5
grid [button $w.pset -text "set" -state disabled -command {setpos_cb}] -row 3 -column 4 -sticky nwse -pady 5 -padx 5
bind $w.pval <Return> {setpos_cb}
grid [radiobutton $w.vel -text "Velocity test" -variable test -value 3 -command test_cb] -row 4 -column 0 -columnspan 3 -sticky w
grid [label $w.vlbl -text "velocity (-265..265)" -state disabled] -row 5 -column 1 -columnspan 2 -sticky e
grid [spinbox $w.vval -width 6 -state disabled -textvariable vel -from -265 -to 265 -increment 10 -repeatdelay 1000 -repeatinterval 500] -row 5 -column 3 -sticky nswe -pady 5 -padx 5
grid [button $w.vset -text "set" -state disabled -command {setvel_cb}] -row 5 -column 4 -sticky nwse -pady 5 -padx 5
bind $w.vval <Return> {setvel_cb}
grid columnconfigure $w "all" -minsize 10 -uniform a -weight 1
#grid rowconfigure $w "all" -minsize 8 -uniform a -weight 1

.nb add $w -text "Motor test"

### about page ###############################################
set w [frame .nb.about]
#.nb add [frame .nb.about] -text "About"
pack [label $w.lbl -text "Dynamixel motor control\n\nversion 1.0\n\n(C) 2022 Eric Boucharé"] -fill both -expand yes
.nb add $w -text "About"

### Link status label ########################################
pack [label .status -borderwidth 1 -relief sunken -anchor w -textvariable status] -side bottom -fill x -expand false


############################################################################
# Serial link setup
############################################################################
# serial reader callback
proc reader {serial} {
	if {[set rc [gets $serial data]] == -1} {
		if {[eof $serial]} {
			fileevent $serial r {}
			close $serial
		}
		return
	}

	puts $data
	if {[regexp {X([0-9,]+)} $data -> s]} {
		.nb.setup.idlist.menu delete 0 end
		.nb.test.idlist.menu delete 0 end
		set l [split $s ,]
		puts $l
		foreach k $l {
			.nb.setup.idlist.menu add radiobutton -label $k -variable id
			.nb.test.idlist.menu add radiobutton -label $k -variable id
		}
	}
}

# init serial
if [catch {
	set serial [open /dev/ttyACM0 r+]
#	fconfigure $serial -mode 115200,n,8,1 -buffering none -blocking 0 -encoding ascii -translation binary
	fconfigure $serial -mode 115200,n,8,1 -blocking 1 -encoding ascii -translation auto
	fileevent $serial readable [list reader $serial]
	set status "Dynamixel control : serial link /dev/ttyACM0,8N1,115200 bauds"
} rc] {
	set status "Dynamixel control : serial link /dev/ttyACM0 could not be opened ..."
}
