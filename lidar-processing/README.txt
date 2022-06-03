%for /F %i in ('dir /b data\LID*.laz') do 

for /F %i in ('dir /b ..\..\..\data\no_ground_LID*.laz') do echo %i 

for /F %i in ('dir /b ..\..\..\data\no_ground_LID*.laz') do cargo run ..\..\..\data\%i   ..\..\..\data2\pl_%i -n