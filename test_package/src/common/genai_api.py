from openai import OpenAI
import os
"""
Performs the necessary AI prompts
"""
class GenerateAISpeech():
    def __init__(self):
        self.client = OpenAI(api_key = os.getenv('GOOGLE_GEMINI_API_KEY'),
                             base_url = "https://generativelanguage.googleapis.com/v1beta/openai/")
        self.conversationHistory = ""

    """
    Used to generate greeting to user
    """
    def generateGreeting(self):
        prompt = (
            "You are LibraGo, a robot librarian. Your job is to greet users and tell them about your capabilities. "
            "LibraGo can retrieve any book from the library that the user asks for as well as engage in conversations about books, journals and any content found in libraries. Keep it succinct and tell the person to wait for the beep that will come in the next few seconds. Do not say the beep itself in your message."
        )
        response = self.client.chat.completions.create(
            model="gemini-1.5-flash",
            messages=[
                {"role": "system", "content": prompt},
                {"role": "user", "content": "Greet me!"}
            ],
        )
        greeting = response.choices[0].message.content
        return greeting

    """
    Used to handle conversational elements/ get a book name   
    """
    def generateResponse(self,content):
        conversationPrompt = f"""You are LibraGo, a robot librarian designed to assist users with inquiries related to books, journals, and other library content. Your primary functions are:
                                            1.Engage in conversations about books, journals, and any content found in libraries in a succinct and efficient manner, offering relevant information or responses as needed.
                                            2.If a user explicitly requests a book to be picked, identify the title of the book and respond with the 'WANTS BOOK: name of the book'.
                                            3.If the user states that they would like more than one book or the book title they have said is incorrect, return INVALID ARGS.
                                            Please refer to the following conversation history for context: {self.conversationHistory}"""
        response = self.client.chat.completions.create(
            model="gemini-1.5-flash",
            messages=[
                {"role": "system", "content": conversationPrompt},
                {"role": "user", "content": content}
            ],
        )
        responseContent = response.choices[0].message.content
        return responseContent
    

    """
    Used in handling success or failure of book delivery
    """
    def generateSuccessOrFailMessage(self,bookName,successfulFind):
        successFailureMessagePrompt = "You are LibraGo, a robot librarian. You attempted to find a book in a library and deliver it and you were {successType} in doing so. " \
                                      "Deliver this news to the individual who requested it for the following book, you are speaking to their face. " \
                                      "Do not say beep boop or use speech marks or * in your output."

        if successfulFind:
            prompt = (
                successFailureMessagePrompt.format(successType = "successful")
            )
        else:
            prompt = (
                successFailureMessagePrompt.format(successType = "unsuccessful")
            )
        response = self.client.chat.completions.create(
            model="gemini-1.5-flash",

            messages=[
                {"role": "system", "content": prompt},
                {"role": "user", "content": bookName}
            ],
        )
        greeting = response.choices[0].message.content
        return greeting

    """
    In charge of holding prompt/conversation history as well as resetting it
    """

    def addToUserConversationHistory(self,msg,wasUser):
        if wasUser:
            speech = f"The user said: {msg} \n"
        else:
            speech = f"You said: {msg} \n"
        self.conversationHistory += speech

    def getConversationHistory(self):
        return self.conversationHistory