FROM hylee101001/slam:p35_g2opy_pangolin
COPY requirements.txt /home/
WORKDIR /home/
RUN python3 -m pip install -r requirements.txt

