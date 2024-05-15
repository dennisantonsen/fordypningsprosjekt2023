FROM python:3.12.3-bookworm

WORKDIR /workdir
COPY ./requirements.txt ./
RUN pip install --no-cache-dir -r requirements.txt