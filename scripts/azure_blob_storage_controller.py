import os
import uuid
from azure.storage.blob import BlobServiceClient


class AzureBlobStorageController:
    def __init__(self, file_path):
        connect_str = os.getenv('AZURE_STORAGE_CONNECTION_STRING')
        self.container_name = os.getenv('AZURE_STORAGE_CONTAINER_NAME')
        self.blob_service_client = BlobServiceClient.from_connection_string(connect_str)
        self.file_path = file_path

    def upload(self, file_name):
        # Write text to the file
        blob_client = self.blob_service_client.get_blob_client(container=self.container_name, blob=file_name)
        print("\nUploading to Azure Storage as blob:\n\t" + file_name)
        upload_file_path = os.path.join(self.file_path, file_name)
        # Upload the created file
        with open(upload_file_path, "rb") as data:
            blob_client.upload_blob(data)
