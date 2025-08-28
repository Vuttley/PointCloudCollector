import zmq

def main():

    context = zmq.Context()

    frontend = context.socket(zmq.XSUB)
    frontend.bind("tcp://*:5559")
    print("Proxy frontend escutando na porta 5559...")

    backend = context.socket(zmq.XPUB)
    backend.bind("tcp://*:5560")
    print("Proxy backend escutando na porta 5560...")

    print("Proxy ZMQ iniciado. Pressione Ctrl+C para encerrar.")
    
    try:
        zmq.proxy(frontend, backend)
    except KeyboardInterrupt:
        print("\nProxy encerrado pelo usuário.")
    finally:
        frontend.close()
        backend.close()
        context.term()

if __name__ == "__main__":
    main()
