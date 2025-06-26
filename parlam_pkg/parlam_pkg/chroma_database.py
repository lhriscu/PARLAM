import os
import pysqlite3
# If working with virtual environments, pyqslite3 might give errors, need to be installed also outside of virtual environment
import sys
sys.modules["sqlite3"] = sys.modules.pop("pysqlite3")
from typing import List
#from langchain.document_loaders import PyPDFLoader
from langchain_community.document_loaders import PyPDFLoader

from langchain.text_splitter import RecursiveCharacterTextSplitter
#from langchain.vectorstores import Chroma
from langchain_chroma import Chroma
import re

class ChromaDB:
    def __init__(self, doc_path: str, db_path: str, embeddings):
        """
        Initialize the PDF Processor with paths and embedding function.

        Args:
            doc_path (str): The path to the PDF file or directory.
            db_path (str): The directory to store the Chroma database.
            embeddings: The embedding function for vector storage.
        """
        self._doc_path = doc_path
        self._db_path = db_path
        self._embeddings = embeddings

    def chunk_and_split_pdf(self, path: str) -> List[str]:
        """
        Load and split PDF documents into chunks of text.

        Args:
            path (str): The path to the PDF file or directory containing PDF files.

        Returns:
            List[str]: A list of text chunks extracted from the PDF documents.
        """
        documents = []
        if os.path.isdir(path):
            for file in os.listdir(path):
                if file.endswith('.pdf'):
                    pdf_path = os.path.join(path, file)
                    loader = PyPDFLoader(pdf_path)
                    documents.extend(loader.load())
        elif os.path.isfile(path):
            loader = PyPDFLoader(path)
            documents = loader.load()
        else:
            raise ValueError('Not a valid path or file. Please provide an existing path or PDF file.')
        
        text_splitter = RecursiveCharacterTextSplitter(chunk_size=500, chunk_overlap=150)

        # Clean documents
        for doc in documents:
            doc.page_content = re.sub(r"(?<!\n)\n(?!\n)", " ", doc.page_content) 
            doc.page_content = re.sub(r"\n\n+", "\n", doc.page_content)

        #text_splitter = NLTKTextSplitter()        
        chunks = text_splitter.split_documents(documents)

        return chunks

    def chroma_database(self) -> Chroma:
        """
        Create or load a Chroma database for similarity search.

        Returns:
            Chroma: The Chroma database.
        """
        # Initialize or load Chroma database
        db = Chroma(persist_directory=self._db_path, embedding_function=self._embeddings)
        
        # Check if the database is empty; if so, load documents and populate it
        if len(db.get()['documents']) == 0:
            documents = self.chunk_and_split_pdf(self._doc_path)
            db = Chroma.from_documents(documents, self._embeddings, persist_directory=self._db_path)
        
        return self.chunk_and_split_pdf(self._doc_path), db

