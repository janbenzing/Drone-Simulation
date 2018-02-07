#!/bin/bash

if [ -t 1 ] ; then
echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
echo "Hint: to show formatted results from command line, install lynx and run: dronecourse_make_report.sh | lynx --stdin"
echo "~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~"
fi

cat <<EOF
<html>
<head>
<title>Report</title>
</head>
<body>
<table>
<thead>
<tr>
<th>SCIPER</th>
<th>Grade</th>
<th>UUID</th>
<th>WP#1</th>
<th>WP#2</th>
<th>WP#3</th>
</tr>
</thead>
<tbody>
EOF

if [ ! -d reports/latest ]; then
echo "Error: reports/latest is not a directory"
fi

for f in reports/latest/*; do
if [ $(basename $f) == "students" ]; then continue; fi
grade=0
uuid=$(basename $f | sed s/....$//)
sciper=$(grep $uuid reports/latest/students | cut -d' ' -f2)
wp0=$(cat $f | sed -n 's/^SUP: wp 0 1$/1/p' | wc -l)
wp1=$(cat $f | sed -n 's/^SUP: wp 1 1$/1/p' | wc -l)
wp2=$(cat $f | sed -n 's/^SUP: wp 2 1$/1/p' | wc -l)

grade=$(echo $wp0+$wp1+$wp2|bc)

echo -n "<tr>"
echo -n "<td>$sciper</td>"
echo -n "<td>$grade</td>"
echo -n "<td>$uuid</td>"
echo -n "<td>$wp0</td>"
echo -n "<td>$wp1</td>"
echo -n "<td>$wp2</td>"
echo "</tr>"
done

cat <<EOF
</tbody>
</body>
</html>
EOF
