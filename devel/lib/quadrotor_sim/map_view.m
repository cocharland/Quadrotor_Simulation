% Takes the generated observaation map and plots it to be checked
close all

data = csvread('Test.txt');
image(data,'CDataMapping','scaled')


data = csvread('TestCol.txt');
sum(data > 0,'all')
figure
image(data,'CDataMapping','scaled')

data = csvread('TestUpdate.txt');
figure
image(data,'CDataMapping','scaled')
