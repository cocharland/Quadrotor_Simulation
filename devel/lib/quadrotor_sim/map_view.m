% Takes the generated observaation map and plots it to be checked
close all

data = csvread('Test.txt');
image(data,'CDataMapping','scaled')

data = csvread('TestCol.txt');
figure
image(data,'CDataMapping','scaled')
