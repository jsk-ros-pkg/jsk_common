
TOPICS=$(rostopic list)
SERVS=$(rosservice list)

echo '------------------------------------------------------------'
echo '- topics                                                   -'
echo '------------------------------------------------------------'
for tpk in $TOPICS; do
    echo ";; topic name $tpk"
    rostopic type $tpk
    rosmsg show $(rostopic type $tpk)
    rostopic info $tpk
    echo '------------------------------------------------------------'
done

echo '------------------------------------------------------------'
echo '- servicess                                                -'
echo '------------------------------------------------------------'
for srv in $SERVS; do
    echo ";; service name $srv"
    rosservice type $srv
    rossrv show $(rosservice type $srv)
    rosservice info $srv
    echo '------------------------------------------------------------'
done