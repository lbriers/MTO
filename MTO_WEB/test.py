import website
import time
import threading

def update_website_angles():
    while True: 
        website_data = website.get_web_angles()

        time.sleep(0.1)

def run_website():
    website.start_server()
    update_website_angles()

        

def main():
    
    #run the website
    website_thread = threading.Thread(target=run_website, daemon=True)
    website_thread.start()

    website_thread.join()

if __name__ == "__main__":
    main()
