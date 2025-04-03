import speech_recognition as sr
from fuzzywuzzy import fuzz
recogniser = sr.Recognizer()
simple_file = open("test/audio_tests/audio_files/simple_phrases/phrases_registered.txt","r")
complex_file = open("test/audio_tests/audio_files/complex_phrases/phrases_registered.txt","r")

simple_arr = []
complex_arr = []

simple_arr_success = {'100': 0,'g95': 0,'g90':0,'else': 0}
complex_arr_success = {'100': 0,'g95': 0,'g90':0,'else': 0}

def read_simple_file():
    for line in simple_file:
        simple_arr.append(line)

def read_complex_file():
    for line in complex_file:
        complex_arr.append(line)


def run_tests():
    read_simple_file()
    read_complex_file()
    simple_file.close()
    complex_file.close()

    for i in range(50):
        with sr.AudioFile(f'test/audio_tests/audio_files/simple_phrases/{i}.wav') as source:
            audio_data = recogniser.record(source)
        try:
            understoodSentence = recogniser.recognize_google(audio_data,language="en-US")
            similarityScore = fuzz.token_sort_ratio(understoodSentence.lower(),simple_arr[i].lower())
            print(f"Test {i} score == {similarityScore}")
            if similarityScore == 100:
                simple_arr_success['100'] = simple_arr_success['100'] + 1
            elif similarityScore >= 95:
                simple_arr_success['g95'] = simple_arr_success['g95'] + 1
            elif similarityScore >= 90:
                simple_arr_success['g90'] = simple_arr_success['g90'] + 1
            else:
                simple_arr_success['else'] = simple_arr_success['else'] + 1

        except sr.UnknownValueError:
            print("err")
        except sr.RequestError as e:
            print("error {0}".format(e))



    for i in range(50):
        with sr.AudioFile(f'test/audio_tests/audio_files/complex_phrases/{i}.wav') as source:
            audio_data = recogniser.record(source)
        try:
            understoodSentence = recogniser.recognize_google(audio_data,language="en-US")
            similarityScore = fuzz.token_sort_ratio(understoodSentence.lower(),complex_arr[i].lower())
            print(f"Test {i} score == {similarityScore}")
            if similarityScore >= 100:
                complex_arr_success['100'] = complex_arr_success['100'] + 1
            elif similarityScore >= 95:
                complex_arr_success['g95'] = complex_arr_success['g95'] + 1
            elif similarityScore >= 90:
                complex_arr_success['g90'] = complex_arr_success['g90'] + 1
            else:
                complex_arr_success['else'] = complex_arr_success['else'] + 1
                
        except sr.UnknownValueError:
            print("err")
        except sr.RequestError as e:
            print("error {0}".format(e))
    
    print(f"Simple arr success: {simple_arr_success}")
    print(f"Complex arr success: {complex_arr_success}")

if __name__=="__main__":
    run_tests()