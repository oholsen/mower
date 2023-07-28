set -x
HOST=$1

( git rev-parse HEAD; git status . ) > version
find edge_control -name '*.py' | tar cf - --files-from - version | ssh $HOST tar Cxvf edge-control -
