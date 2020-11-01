FROM buildpack-deps:stretch-curl

ENV DIRPATH /opt/app
WORKDIR $DIRPATH

RUN apt-get update && apt-get install -y --no-install-recommends \
    unzip

RUN curl -fsSL https://deno.land/x/install/install.sh | sh
ENV DENO_INSTALL /root/.deno
ENV PATH $DENO_INSTALL/bin:$PATH

COPY . $DIRPATH


ENTRYPOINT deno run --allow-env --allow-net index.ts
