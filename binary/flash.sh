if [[ $# -eq 0 ]]
then
	echo "no file name provided"
	exit 1
fi
if ! [[ -f $1 ]]
then
	echo "file not found"
	exit 2
fi

mspdebug tilib "prog $1" --allow-fw-update
