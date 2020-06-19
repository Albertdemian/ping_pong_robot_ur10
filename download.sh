# cd ~/Downloads/
# sudo apt-get update -y
# sudo apt-get install -y swig
# wget https://www.baslerweb.com/fp-1589378344/media/downloads/software/pylon_software/pylon_6.1.1.19861-deb0_amd64.deb
# sudo apt-get install ./pylon_6.1.1.19861-deb0_amd64.deb
# sudo apt-get install python3-pip
#Create output file, override if already present  
output=requirements.txt 
echo "numpy
pylon
matplotlib
imutils
opencv-python
rospkg
" > $output
pip3 install -r requirements.txt
rm -f requirements.txt
