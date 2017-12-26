# DO NOT EDIT
# This makefile makes sure all linkable targets are
# up-to-date with anything they link to
default:
	echo "Do not invoke directly"

# Rules to remove targets that are older than anything to which they
# link.  This forces Xcode to relink the targets from scratch.  It
# does not seem to check these dependencies itself.
PostBuild.mpc.Debug:
/Users/tanwang/Downloads/CarND-MPC-Project-master/Debug/mpc:
	/bin/rm -f /Users/tanwang/Downloads/CarND-MPC-Project-master/Debug/mpc


PostBuild.mpc.Release:
/Users/tanwang/Downloads/CarND-MPC-Project-master/Release/mpc:
	/bin/rm -f /Users/tanwang/Downloads/CarND-MPC-Project-master/Release/mpc


PostBuild.mpc.MinSizeRel:
/Users/tanwang/Downloads/CarND-MPC-Project-master/MinSizeRel/mpc:
	/bin/rm -f /Users/tanwang/Downloads/CarND-MPC-Project-master/MinSizeRel/mpc


PostBuild.mpc.RelWithDebInfo:
/Users/tanwang/Downloads/CarND-MPC-Project-master/RelWithDebInfo/mpc:
	/bin/rm -f /Users/tanwang/Downloads/CarND-MPC-Project-master/RelWithDebInfo/mpc




# For each target create a dummy ruleso the target does not have to exist
